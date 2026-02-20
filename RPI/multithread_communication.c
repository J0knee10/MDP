#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <curl/curl.h>
#include <time.h> // For pthread_cond_timedwait
#include <errno.h> // For ETIMEDOUT

#include "shared_types.h"
#include "rpi_hal.h"
#include "json_parser.h" // New include

// Definition for DIR_MAP_ANDROID_STR, declared in shared_types.h
const char* DIR_MAP_ANDROID_STR[8] = {
    "N", "NE", "E", "SE", "S", "SW", "W", "NW"
};

// --- Configuration ---
#ifdef RPI_TESTING
const char* STM32_DEVICE = "rpi_to_stm";
const char* ANDROID_DEVICE = "android_to_rpi";
const char* PATHFINDING_SERVER_URL = "http://192.168.22.26:4000/path";
const char* IMAGE_SERVER_URL = "http://192.168.22.26:5000/detect";
#elif defined(FAKE_ANDROID_SIMULATION)
const char* STM32_DEVICE = "/dev/ttyACM0";
const char* ANDROID_DEVICE = "android_to_rpi";
const char* PATHFINDING_SERVER_URL = "http://192.168.22.24:5000/path";
const char* IMAGE_SERVER_URL = "http://192.168.22.21:5000/detect";
#else
const char* STM32_DEVICE = "/dev/ttyACM0";
const char* ANDROID_DEVICE = "/dev/rfcomm0";
const char* PATHFINDING_SERVER_URL = "http://192.168.22.24:5000/path";
const char* IMAGE_SERVER_URL = "http://192.168.22.21:5000/detect";
#endif

const int BAUD_RATE = 115200;
const char* CAPTURE_FILENAME = "capture.jpg";

// --- Global Shared Application Context ---
SharedAppContext g_app_context;


// =================================================================================
// THREAD 3: Image Processing (Temporary, "Fire-and-Forget")
// =================================================================================
// Updated post_image_to_server_thread to return response for parsing
static int post_image_to_server_thread(int obstacle_id, char* response_buffer, int buffer_size) {
    CURL* curl;
    CURLcode res;
    int result = -1;

    struct MemoryStruct chunk = { .memory = malloc(1), .size = 0 };
    if (chunk.memory == NULL) { // Check for malloc failure
        fprintf(stderr, "[ImgThread] Failed to allocate memory for CURL response.\n");
        return -1;
    }

    curl = curl_easy_init();
    if (curl) {
        curl_mime *form = curl_mime_init(curl);
        curl_mimepart *field;

        field = curl_mime_addpart(form); curl_mime_name(field, "image"); curl_mime_filedata(field, CAPTURE_FILENAME);
        char id_str[10]; snprintf(id_str, sizeof(id_str), "%d", obstacle_id);
        field = curl_mime_addpart(form); curl_mime_name(field, "object_id"); curl_mime_data(field, id_str, CURL_ZERO_TERMINATED);

        curl_easy_setopt(curl, CURLOPT_URL, IMAGE_SERVER_URL);
        curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);

        res = curl_easy_perform(curl);
        if (res == CURLE_OK) {
            long code; curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
            if (code >= 200 && code < 300) {
                if (response_buffer != NULL && chunk.memory != NULL) { // Only copy if buffer provided and memory exists
                    strncpy(response_buffer, chunk.memory, buffer_size - 1);
                    response_buffer[buffer_size - 1] = '\0';
                }
                result = 0;
            } else {
                fprintf(stderr, "[ImgThread] Image server returned non-2xx response: %ld\n", code);
            }
        } else {
            fprintf(stderr, "[ImgThread] post_image_to_server_thread failed: %s\n", curl_easy_strerror(res));
        }
        curl_easy_cleanup(curl);
        curl_mime_free(form);
    } else {
        fprintf(stderr, "[ImgThread] curl_easy_init() failed.\n");
    }
    free(chunk.memory); // Free after curl_easy_cleanup
    return result;
}

void* process_image_thread(void* args) {
    ImageTaskArgs* task_args = (ImageTaskArgs*)args;
    SharedAppContext* context = task_args->context;
    char image_server_response[2048]; // Buffer for image server JSON response
    char class_label[100]; // To hold the detected class label

    printf("[ImgThread] Capturing image for obstacle %d...\n", task_args->obstacle_id);
    if (capture_image(CAPTURE_FILENAME) != 0) {
        fprintf(stderr, "[ImgThread] Failed to capture image.\n");
        // Signal image capture failure by setting ID to 0 or another error code, or just don't signal
        pthread_mutex_lock(&context->image_capture_mutex);
        context->last_image_capture_id = 0; // Indicate failure or no successful capture
        pthread_cond_signal(&context->image_capture_cond);
        pthread_mutex_unlock(&context->image_capture_mutex);
    } else {
        printf("[ImgThread] Image captured successfully for obstacle %d.\n", task_args->obstacle_id);
        // Signal image capture success
        pthread_mutex_lock(&context->image_capture_mutex);
        context->last_image_capture_id = task_args->obstacle_id;
        pthread_cond_signal(&context->image_capture_cond);
        pthread_mutex_unlock(&context->image_capture_mutex);

        // Send robot position to Android (Python's ROBOT,x,y,d)
        char robot_pos_msg[100];
        // Use +1 for x and y to match Python's 1-indexed coordinates for Android
        const char* dir_str = (task_args->robot_snap_position.d >= 0 && task_args->robot_snap_position.d < 8) ?
                               DIR_MAP_ANDROID_STR[task_args->robot_snap_position.d] : "U"; // U for unknown
        snprintf(robot_pos_msg, sizeof(robot_pos_msg), "\"ROBOT,%d,%d,%s\"\n",
                 task_args->robot_snap_position.x + 1, task_args->robot_snap_position.y + 1, dir_str);
        send_message_to_android_with_ack(context->android_fd, robot_pos_msg);
        printf("[ImgThread] Sent robot position to Android: %s", robot_pos_msg);


        // Post image and get response
        if (post_image_to_server_thread(task_args->obstacle_id, image_server_response, sizeof(image_server_response)) == 0) {
            printf("[ImgThread] Image server response: %s\n", image_server_response);

            /* Compatible with object_detection_server.py: server returns success, detected, count, objects[] with class_label, img_id, confidence, bbox.
             * Use "count" for detection (integer); prefer "img_id" from JSON; do not skip Bullseye — use first object with valid img_id. */
            int count = 0;
            if (get_json_int(image_server_response, "count", &count) != 0 || count <= 0) {
                printf("[ImgThread] No object detected by image server for obstacle %d.\n", task_args->obstacle_id);
            } else {
                const char* objects_array_start = strstr(image_server_response, "\"objects\":[");
                if (objects_array_start) {
                    objects_array_start += strlen("\"objects\":[");
                    const char* ptr = objects_array_start;
                    int sent = 0;
                    while (*ptr && sent == 0) {
                        const char* obj_start = strchr(ptr, '{');
                        if (!obj_start) break;
                        int depth = 1;
                        const char* p = obj_start + 1;
                        while (*p && depth > 0) {
                            if (*p == '{') depth++;
                            else if (*p == '}') depth--;
                            p++;
                        }
                        if (depth != 0) break;
                        const char* obj_end = p - 1;
                        size_t obj_len = (size_t)(obj_end - obj_start + 1);
                        char single_obj_json[512];
                        if (obj_len >= sizeof(single_obj_json)) obj_len = sizeof(single_obj_json) - 1;
                        strncpy(single_obj_json, obj_start, obj_len);
                        single_obj_json[obj_len] = '\0';

                        if (get_json_string(single_obj_json, "class_label", class_label, sizeof(class_label)) != 0)
                            get_json_string(single_obj_json, "class", class_label, sizeof(class_label));
                        if (class_label[0] != '\0') {
                            /* Strip " - ..." suffix if present (server may send "Number 4 - 4") */
                            char* dash = strstr(class_label, " - ");
                            if (dash) *dash = '\0';
                            int img_id = -1;
                            if (get_json_int(single_obj_json, "img_id", &img_id) != 0 || img_id < 0)
                                img_id = get_img_id_from_class_name(class_label);
                            if (img_id >= 0) {
                                /* Send TARGET,object_id,img_id to Android (e.g. "TARGET,1,11") */
                                send_target_result_to_android(context->android_fd, task_args->obstacle_id, img_id);
                                printf("[ImgThread] Sent image detection result to Android: obstacle_id=%d, class_label=%s, img_id=%d\n", task_args->obstacle_id, class_label, img_id);
                                sent = 1;
                            } else {
                                fprintf(stderr, "[ImgThread] Unknown class label received or invalid img_id: %s\n", class_label);
                            }
                        }
                        ptr = p;
                    }
                    if (sent == 0)
                        fprintf(stderr, "[ImgThread] No valid object with img_id for obstacle %d.\n", task_args->obstacle_id);
                }
            }
        } else {
            fprintf(stderr, "[ImgThread] Failed to upload image or no ACK received from image server.\n");
        }
    }
    free(task_args); // Free the dynamically allocated arguments
    return NULL;
}


// =================================================================================
// THREAD 2: Navigation Executor (Main Logic)
// =================================================================================

void execute_navigation() {
    SharedAppContext* context = &g_app_context;
    printf("[NavThread] State: [NAVIGATING]. Executing %d commands.\n", context->command_count);

    pthread_mutex_lock(&context->lock);
    context->snap_position_idx = 0; // Reset snap position index for new navigation
    pthread_mutex_unlock(&context->lock);

    uint32_t current_cmd_id = 1; // Start command IDs from 1 for the sequence

    for (int i = 0; i < context->command_count; i++) {
        pthread_mutex_lock(&context->lock);
        if (context->stop_requested) {
            printf("[NavThread] Stop requested. Aborting navigation.\n");
            context->stop_requested = false;
            context->state = STATE_IDLE;
            pthread_mutex_unlock(&context->lock);
            break;
        }
        pthread_mutex_unlock(&context->lock);

        Command cmd = context->commands[i];
        if (cmd.type == CMD_SNAPSHOT) {
            printf("[NavThread] --- Spawning image thread for obstacle %d ---\n", cmd.value);
            pthread_t tid;
            ImageTaskArgs* args = malloc(sizeof(ImageTaskArgs));
            if (!args) {
                fprintf(stderr, "[NavThread] Failed to allocate ImageTaskArgs.\n");
                continue;
            }
            args->context = context;
            args->obstacle_id = cmd.value;
            // Get current snap position from context
            pthread_mutex_lock(&context->lock);
            if (context->snap_position_idx < context->snap_position_count) {
                args->robot_snap_position = context->snap_positions[context->snap_position_idx];
                context->snap_position_idx++;
            } else {
                // Fallback if snap positions don't match commands, should not happen with correct parsing
                args->robot_snap_position = (SnapPosition){.x = -1, .y = -1, .d = -1};
                fprintf(stderr, "[NavThread] Warning: Snap position index out of bounds.\n");
            }
            pthread_mutex_unlock(&context->lock);


            pthread_create(&tid, NULL, process_image_thread, args);
            pthread_detach(tid); // Detach to allow thread to clean up its resources automatically

            printf("[NavThread] Spawning image thread for obstacle %d. Waiting for image capture confirmation...\n", cmd.value);

            struct timespec ts_img;
            clock_gettime(CLOCK_REALTIME, &ts_img);
            ts_img.tv_sec += 10; // Wait for up to 10 seconds for image capture confirmation

            int img_ack_result = 0; // 0 for success, -1 for error/timeout
            pthread_mutex_lock(&context->image_capture_mutex);
            while (context->last_image_capture_id != (uint32_t)cmd.value && !context->stop_requested) {
                int rc = pthread_cond_timedwait(&context->image_capture_cond, &context->image_capture_mutex, &ts_img);
                if (rc == ETIMEDOUT) {
                    fprintf(stderr, "[NavThread] Timeout waiting for image capture confirmation for obstacle %d.\n", cmd.value);
                    img_ack_result = -1; // Indicate error
                    break;
                } else if (rc != 0) {
                    fprintf(stderr, "[NavThread] Error waiting for image capture condition variable: %d\n", rc);
                    img_ack_result = -1; // Indicate error
                    break;
                }
            }

            if (img_ack_result == 0 && context->last_image_capture_id == (uint32_t)cmd.value) {
                printf("[NavThread] Received image capture confirmation for obstacle %d. Proceeding.\n", cmd.value);
            } else if (img_ack_result == 0 && context->last_image_capture_id == 0) {
                // This means an image capture failed (last_image_capture_id was set to 0)
                fprintf(stderr, "[NavThread] Image capture for obstacle %d indicated failure. Aborting navigation.\n", cmd.value);
                img_ack_result = -1; // Treat as failure for navigation flow
            }
            pthread_mutex_unlock(&context->image_capture_mutex);

            if (img_ack_result == -1 || context->stop_requested) {
                // If there was an error or stop was requested while waiting, break out of navigation
                pthread_mutex_lock(&context->lock);
                context->stop_requested = true; // Ensure stop state is propagated
                context->state = STATE_IDLE;
                pthread_mutex_unlock(&context->lock);
                break; // Exit the command execution loop
            }


        } else {
            // Send command to STM32 with a sequential ID
            uint32_t sent_cmd_id = current_cmd_id++; // Store the ID we're sending
            send_command_to_stm32(context->stm32_fd, cmd, sent_cmd_id);
            printf("[NavThread] Sent command %u to STM32. Waiting for ACK...\n", sent_cmd_id);

            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 10; // Wait for up to 5 seconds for ACK

            int ack_result = 0; // 0 for success, -1 for error/timeout
            pthread_mutex_lock(&context->stm32_ack_mutex);
            while (context->stm32_last_ack_id != sent_cmd_id && !context->stop_requested) {
                int rc = pthread_cond_timedwait(&context->stm32_ack_cond, &context->stm32_ack_mutex, &ts);
                if (rc == ETIMEDOUT) {
                    fprintf(stderr, "[NavThread] Timeout waiting for ACK for command %u.\n", sent_cmd_id);
                    ack_result = -1; // Indicate error
                    break;
                } else if (rc != 0) {
                    fprintf(stderr, "[NavThread] Error waiting for ACK condition variable: %d\n", rc);
                    ack_result = -1; // Indicate error
                    break;
                }
            }

            if (ack_result == 0 && context->stm32_last_ack_id == sent_cmd_id) {
                printf("[NavThread] Received ACK for command %u.\n", sent_cmd_id);
            }
            pthread_mutex_unlock(&context->stm32_ack_mutex);

            if (ack_result == -1 || context->stop_requested) {
                // If there was an error or stop was requested while waiting, break out of navigation
                pthread_mutex_lock(&context->lock);
                context->stop_requested = true; // Ensure stop state is propagated
                context->state = STATE_IDLE;
                pthread_mutex_unlock(&context->lock);
                break; // Exit the command execution loop
            }
        }
    } // End of for loop
    // Using send_message_to_android_with_ack for navigation completion status
    send_message_to_android_with_ack(context->android_fd, "\"Navigation complete.\"\n");
}

void* navigation_executor_thread(void* args) {
    SharedAppContext* context = (SharedAppContext*)args;

    while (1) {
        pthread_mutex_lock(&context->lock);
        while (!context->new_map_received && !context->stop_requested) {
            printf("[NavThread] State: [IDLE]. Waiting for new mission...\n");
            pthread_cond_wait(&context->new_task_cond, &context->lock);
        }

        if (context->stop_requested) {
            context->state = STATE_IDLE;
            context->stop_requested = false;
        }

        if (context->new_map_received) {
            context->state = STATE_PATHFINDING;
            context->new_map_received = false;
        }
        pthread_mutex_unlock(&context->lock);

        if (context->state == STATE_PATHFINDING) {
            printf("[NavThread] State: [PATHFINDING]. Requesting route from server...\n");
            char payload[2048];
            char obstacles_str[1500] = ""; // To build the obstacles array string

            for (int i = 0; i < context->obstacle_count; i++) {
                char obs_item[100]; // Buffer for a single obstacle JSON object
                // Obstacle x, y are 0-indexed internally, server expects 0-indexed
                // Direction 'd' is integer, server expects integer
                snprintf(obs_item, sizeof(obs_item), "{\"id\":%d,\"x\":%d,\"y\":%d,\"d\":%d}",
                         context->obstacles[i].id, context->obstacles[i].x, context->obstacles[i].y, context->obstacles[i].d);
                strcat(obstacles_str, obs_item);
                if (i < context->obstacle_count - 1) strcat(obstacles_str, ",");
            }

            // Construct the full payload including robot initial state and retrying flag
            snprintf(payload, sizeof(payload), "{\"obstacles\":[%s],\"robot_x\":%d,\"robot_y\":%d,\"robot_dir\":%d,\"retrying\":false}",
                     obstacles_str, context->robot_start_x, context->robot_start_y, context->robot_start_dir);
            printf("[NavThread] Pathfinding payload: %s\n", payload);

            char response[4096]; // Increased response buffer size
            if (post_data_to_server(PATHFINDING_SERVER_URL, payload, response, sizeof(response)) == 0) {
                // --- DEBUG: Print raw server response ---
                printf("[NavThread] Raw server response:\n---\n%s\n---\n", response);

                // Call the modified parse_command_route_from_server
                if (parse_command_route_from_server(response, context->commands, &context->command_count,
                                                    context->snap_positions, &context->snap_position_count) == 0) {
                    send_message_to_android_with_ack(context->android_fd, "\"Route calculated. Navigating.\"\n"); // Using ack send
                    execute_navigation();
                } else {
                    send_message_to_android_with_ack(context->android_fd, "\"Error: Pathfinding failed to parse route.\"\n"); // Using ack send
                }
            } else {
                send_message_to_android_with_ack(context->android_fd, "\"Error: Pathfinding server communication failed.\"\n"); // Using ack send
            }
        }

        pthread_mutex_lock(&context->lock);
        context->state = STATE_IDLE;
        pthread_mutex_unlock(&context->lock);
    }
    return NULL;
}
            
            
            // =================================================================================
            // THREAD 1: Android Listener (High-level Commands)
            // =================================================================================
            
            void* android_listener_thread(void* args) {
                SharedAppContext* context = (SharedAppContext*)args;
                char buffer[8192]; // Buffer for incoming Android messages
            
                while (1) {
                    printf("[AndroidThread] Listening for messages...\n");
                    // Blocking read until a message is received
                    ssize_t bytes_read = read(context->android_fd, buffer, sizeof(buffer) - 1);
            
                    if (bytes_read > 0) {
                        buffer[bytes_read] = '\0';
                        printf("[AndroidThread] Received: %s\n", buffer);
            
                        // Check for JSON message first
                        char category[50];
                        if (get_json_string(buffer, "cat", category, sizeof(category)) == 0) {
                            if (strcmp(category, "sendArena") == 0) {
                                const char* value_ptr = strstr(buffer, "\"value\":");
                                if (value_ptr) {
                                    const char* map_json_start = strchr(value_ptr, '{');
                                    if (map_json_start) {
                                        pthread_mutex_lock(&context->lock);
                                        if (context->state == STATE_IDLE) {
                                            if (parse_android_map_and_obstacles(map_json_start, context) == 0) {
                                                context->new_map_received = true;
                                                send_android_ack(context->android_fd, category, "Map received. Pathfinding...");
                                                pthread_cond_signal(&context->new_task_cond);
                                            } else {
                                                send_android_ack(context->android_fd, category, "Error: Invalid map format.");
                                            }
                                        } else {
                                            send_android_ack(context->android_fd, category, "Error: Robot is busy. Cannot start new mission.");
                                        }
                                        pthread_mutex_unlock(&context->lock);
                                    } else {
                                        fprintf(stderr, "[AndroidThread] Malformed 'sendArena': 'value' object not found.\n");
                                        send_android_ack(context->android_fd, category, "Error: Malformed 'sendArena' message.");
                                    }
                                } else {
                                    fprintf(stderr, "[AndroidThread] Malformed 'sendArena': 'value' key not found.\n");
                                    send_android_ack(context->android_fd, category, "Error: Malformed 'sendArena' message.");
                                }
                            } else if (strcmp(category, "stop") == 0) { // STOP command as JSON
                                pthread_mutex_lock(&context->lock);
                                send_android_ack(context->android_fd, category, "STOP command received.");
                                context->stop_requested = true;
                                if(context->state != STATE_IDLE) {
                                    pthread_cond_signal(&context->new_task_cond);
                                }
                                pthread_mutex_unlock(&context->lock);
                            } else if (strcmp(category, "stm") == 0) { // Direct STM command from Android
                                char stm_command_str[100]; // Buffer for the command string like "<FR090>"
                                if (get_json_string(buffer, "value", stm_command_str, sizeof(stm_command_str)) == 0) {
                                    // Parse and execute the STM command. This function will be in rpi_hal.c
                                    // It will also handle waiting for ACK from STM32
                                    parse_and_execute_android_command(context->stm32_fd, stm_command_str, context);
                                } else {
                                    fprintf(stderr, "[AndroidThread] Malformed 'stm' command: 'value' key not found.\n");
                                    send_android_ack(context->android_fd, category, "Error: Malformed STM command.");
                                }
                            } else {
                                fprintf(stderr, "[AndroidThread] Unrecognized JSON category from Android: %s\n", category);
                            }
                        }
                        // All other messages are considered malformed or unrecognized by AndroidThread
                        else {
                            fprintf(stderr, "[AndroidThread] Malformed or unrecognized message from Android: %s\n", buffer);
                        }
                    } else if (bytes_read == 0) {
                        printf("[AndroidThread] Read 0 bytes, serial port might be closed or empty.\n");
                        usleep(10000); // Small delay to prevent busy-waiting
                    } else {
                        perror("[AndroidThread] Error reading from serial port");
                        usleep(100000); // 100ms
                    }
                }
                return NULL;
            }
            
            
            // =================================================================================
            // New THREAD: STM32 Listener
            // =================================================================================
            void* stm32_listener_thread(void* args) {
                SharedAppContext* context = (SharedAppContext*)args;
                char buffer[256]; // Buffer for incoming STM32 messages
                ssize_t bytes_read;
            
                printf("[STM32Thread] Listening for messages...\n");
            
                while (1) {
                    bytes_read = read(context->stm32_fd, buffer, sizeof(buffer) - 1);
            
                    if (bytes_read > 0) {
                        buffer[bytes_read] = '\0';
                        printf("[STM32Thread] Received: %s", buffer); // Use %s directly, as it might contain \n
            
                        // Check for ACK message format: !cmdId/DONE;
                        uint32_t cmd_id;
                        // Assuming the format is !<cmdId>/DONE;
                        if (sscanf(buffer, "!%u/DONE;", &cmd_id) == 1) {
                            pthread_mutex_lock(&context->stm32_ack_mutex);
                            context->stm32_last_ack_id = cmd_id;
                            pthread_cond_signal(&context->stm32_ack_cond);
                            pthread_mutex_unlock(&context->stm32_ack_mutex);
                            printf("[STM32Thread] Processed ACK for CMD ID: %u\n", cmd_id);
                        } else {
                            fprintf(stderr, "[STM32Thread] Unrecognized message format from STM32: %s\n", buffer);
                        }
                    } else if (bytes_read == 0) {
                        // No data for a while, small delay to prevent busy-waiting
                        usleep(10000); // 10ms
                    } else {
                        perror("[STM32Thread] Error reading from serial port");
                        usleep(100000); // 100ms delay on error
                    }
                }
                return NULL;
            }
            
            
            // =================================================================================
            // Main Function (Initialization and Thread Management)
            // =================================================================================

// =================================================================================
// Main Function (Initialization and Thread Management)
// =================================================================================

int main() {
    curl_global_init(CURL_GLOBAL_ALL); // Initialize curl once for the application lifecycle
    memset(&g_app_context, 0, sizeof(SharedAppContext));
    pthread_mutex_init(&g_app_context.lock, NULL);
    pthread_cond_init(&g_app_context.new_task_cond, NULL);
    g_app_context.state = STATE_IDLE;
    g_app_context.snap_position_count = 0; // Initialize new fields
    g_app_context.snap_position_idx = 0;   // Initialize new fields

    // Initialize STM32 ACK synchronization mechanisms
    g_app_context.stm32_last_ack_id = 0;
    pthread_mutex_init(&g_app_context.stm32_ack_mutex, NULL);
    pthread_cond_init(&g_app_context.stm32_ack_cond, NULL);

    // Initialize Image capture synchronization mechanisms
    g_app_context.last_image_capture_id = 0;
    pthread_mutex_init(&g_app_context.image_capture_mutex, NULL);
    pthread_cond_init(&g_app_context.image_capture_cond, NULL);



    g_app_context.stm32_fd = init_serial_port(STM32_DEVICE, BAUD_RATE);
    g_app_context.android_fd = init_serial_port(ANDROID_DEVICE, BAUD_RATE);

    if (g_app_context.stm32_fd == -1 || g_app_context.android_fd == -1) {
        fprintf(stderr, "Fatal: Failed to initialize serial ports. Exiting.\n");
        return 1;
    }

    printf("--- RPi Control Centre Initialized ---\n");

    pthread_t android_tid, nav_tid, stm32_tid;
    pthread_create(&android_tid, NULL, android_listener_thread, &g_app_context);
    pthread_create(&nav_tid, NULL, navigation_executor_thread, &g_app_context);
    pthread_create(&stm32_tid, NULL, stm32_listener_thread, &g_app_context); // Create the new STM32 listener thread

    pthread_join(android_tid, NULL);
    pthread_join(nav_tid, NULL);
    pthread_join(stm32_tid, NULL); // Join the new STM32 listener thread

    pthread_mutex_destroy(&g_app_context.lock);
    pthread_cond_destroy(&g_app_context.new_task_cond);
    pthread_mutex_destroy(&g_app_context.stm32_ack_mutex); // Destroy new mutex
    pthread_cond_destroy(&g_app_context.stm32_ack_cond);   // Destroy new condition variable
    pthread_mutex_destroy(&g_app_context.image_capture_mutex); // Destroy image capture mutex
    pthread_cond_destroy(&g_app_context.image_capture_cond);   // Destroy image capture condition variable
    close(g_app_context.stm32_fd);
    close(g_app_context.android_fd);

    curl_global_cleanup(); // Clean up curl once at application shutdown
    return 0;
}


// =================================================================================
// Testing Instructions (Without Hardware)
// =================================================================================
/*
To test `multithread_communication.c` without physical Android or STM32 hardware,
but with the fake servers, follow these steps.

**Prerequisites:**
1.  Ensure you have `python3` installed.
2.  Ensure you have `curl` development libraries installed (e.g., `libcurl4-openssl-dev` on Debian/Ubuntu).
3.  Ensure `json_parser.c`, `json_parser.h`, `rpi_hal.c`, `rpi_hal.h`, and `shared_types.h` are in the same directory or accessible via include paths.

**Step 1: Compile the RPI communication module**

Open your terminal in the `RPI` directory and compile with the `RPI_TESTING` flag defined:

    gcc -g -Wall -DRPI_TESTING multithread_communication.c json_parser.c rpi_hal.c -o test_center -lpthread -lcurl
    gcc -g -Wall -DFAKE_ANDROID_SIMULATION multithread_communication.c json_parser.c rpi_hal.c -o STtest_center -lpthread -lcurl
    gcc -Wall multithread_communication.c json_parser.c rpi_hal.c -o ctrl_center -lpthread -lcurl
    Make `fake_stm.py` executable:
        chmod +x fake_stm.py

*   `-DRPI_TESTING`: Activates the test configuration (e.g., using named pipes for STM/Android and localhost for servers).
*   `-o test_center`: Specifies the output executable name.
*   `-lpthread`: Links the POSIX threads library.
*   `-lcurl`: Links the libcurl library.

**Step 2: Create Named Pipes (FIFOs) for simulated serial communication**

The `RPI_TESTING` configuration uses "rpi_to_stm" and "android_to_rpi" as device paths. These need to be created as named pipes.
Run these commands in your terminal in the `RPI` directory:

    mkfifo rpi_to_stm
    mkfifo android_to_rpi

**Step 3: Run the Fake Servers**

Open two SEPARATE terminal windows/tabs, navigate to the `RPI` directory in each, and run the fake servers:

*   **Terminal 1 (Fake Path Server):**
    ```bash
    python3 fake_path_server.py
    ```
    You should see: `Fake Pathfinding Server running on http://localhost:5000 ...`

*   **Terminal 2 (Fake Image Server):**
    ```bash
    python3 fake_image_server.py
    ```
    You should see: `Fake Image Recognition Server running on http://localhost:5000 ...`

*   **Terminal 3 (Fake STM32 Simulation):**
    ```bash
    python3 fake_stm.py
    ```

**Step 4: Run the RPI Communication Program**

Open a THIRD terminal window/tab, navigate to the `RPI` directory, and run the compiled program:

    ./test_center

You should see initialization messages like: `--- RPi Control Centre Initialized ---` and `[AndroidThread] Listening for messages...` and `[NavThread] State: [IDLE]. Waiting for new mission...`

**Step 5: Simulate Android Input (Trigger Pathfinding and Image Processing)**

Open a FOURTH terminal window/tab, navigate to the `RPI` directory.
You will write a "START" command with obstacle data to the `android_to_rpi` named pipe. This simulates the Android app sending a mission.

**Example START Command:**

```json
"{"cat": "sendArena", "value": {"obstacles":[{"x": 9,"y": 9,"d": 0,"id": 0},{"x": 9,"y": 10,"d": 0,"id": 0},{"x": 2,"y": 12,"d": 1,"id": 1},{"x": 12,"y": 17,"d": 2,"id": 2},{"x": 11,"y": 4,"d": 4,"id": 3},{"x": 17,"y": 10,"d": 3,"id": 4}],"robot_x": 1,"robot_y": 1,"robot_direction": 1}}"
```

**To send this command, paste the following into the FOURTH terminal and press Enter:**

    echo "{\"cat\": \"sendArena\", \"value\": {\"obstacles\":[{\"id\":1,\"x\":1,\"y\":2,\"d\":2},{\"id\":2,\"x\":2,\"y\":3,\"d\":0}],\"robot_x\":0,\"robot_y\":0,\"robot_dir\":0,\"retrying\":false}}\\n" > android_to_rpi

*   **Explanation of the Android message format:**
    *   It starts with `"START`
    *   The entire JSON payload is enclosed in double quotes.
    *   `obstacles`: An array of objects, each with `id`, `x`, `y`, `d` (direction).
    *   `robot_x`, `robot_y`, `robot_dir`: Initial robot position and direction.
    *   `retrying`: Boolean flag.

**Expected Output in `rpi_comm` terminal (Step 4):**

You should see the RPI program:
1.  Receive the START message.
2.  Transition to `STATE_PATHFINDING`.
3.  Print the pathfinding payload.
4.  Make a request to `http://localhost:5000/path` (handled by `fake_path_server.py`).
5.  Receive and parse the route (commands and snap positions).
6.  Start `execute_navigation()`.
7.  For each `CMD_SNAPSHOT` command, it will print `--- Spawning image thread for obstacle X ---`.
8.  The image thread will capture image (simulated), post to `http://localhost:5000/detect` (handled by `fake_image_server.py`), and print the image server's response.
9.  It will then simulate sending a robot position and image detection result to Android (these messages will be written to `rpi_to_stm`, but since no one is reading from `rpi_to_stm` in this test, you won't see them directly unless you monitor the pipe).
10. Finally, it will print `"Navigation complete."` and return to `STATE_IDLE`.

**Step 6: Observe STM32 commands (Optional)**

If you want to see what commands are sent to the STM32, open a FIFTH terminal window/tab and run:

    cat rpi_to_stm

This will display the messages that `rpi_comm` sends to the STM32 simulated device. You should see `ACK` messages and command strings (e.g., `:0/FW10;`).

**Step 7: Simulate Android STOP Command**

To simulate stopping the robot mid-navigation (or just sending a stop command while idle), in the FOURTH terminal, type:

    echo "\"STOP\"\\n" > android_to_rpi

The `rpi_comm` program should acknowledge the STOP command. If navigation is in progress, it will abort.

**Cleanup:**

*   Press `Ctrl+C` in all terminal windows running the servers and `rpi_comm`.
*   Remove the named pipes:
    ```bash
    rm rpi_to_stm
    rm android_to_rpi
    ```
*/