#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <curl/curl.h>

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
const char* PATHFINDING_SERVER_URL = "http://127.0.0.1:5000/path";
const char* IMAGE_SERVER_URL = "http://127.0.0.1:5000/detect";
#elif defined(FAKE_ANDROID_SIMULATION)
const char* STM32_DEVICE = "/dev/ttyACM0";
const char* ANDROID_DEVICE = "android_to_rpi";
const char* PATHFINDING_SERVER_URL = "http://127.0.0.1:5000/path";
const char* IMAGE_SERVER_URL = "http://127.0.0.1:5000/detect";
#else
const char* STM32_DEVICE = "/dev/ttyACM0";
const char* ANDROID_DEVICE = "/dev/rfcomm0";
const char* PATHFINDING_SERVER_URL = "http://192.168.22.230:4000/path";
const char* IMAGE_SERVER_URL = "http://192.168.22.7:5000/detect";
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
    } else {
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

            // Parse response to get detected object and send to Android
            int detected = 0;
            // The JSON parsing for "detected" and "objects" array needs to be more robust.
            // For now, using basic string searches as per json_parser.c implementation strategy.
            if (get_json_int(image_server_response, "detected", &detected) == 0 && detected == 1) {
                // Find the "objects" array and then the first object
                const char* objects_array_start = strstr(image_server_response, "\"objects\":[");
                if (objects_array_start) {
                    objects_array_start += strlen("\"objects\":[");
                    const char* first_obj_start = strchr(objects_array_start, '{');
                    if (first_obj_start) {
                        const char* first_obj_end = strchr(first_obj_start, '}');
                        if (first_obj_end) {
                            char single_obj_json[512]; // Temporary buffer for one object's JSON
                            strncpy(single_obj_json, first_obj_start, first_obj_end - first_obj_start + 1);
                            single_obj_json[first_obj_end - first_obj_start + 1] = '\0';

                            // Extract class_label from the object JSON
                            if (get_json_string(single_obj_json, "class_label", class_label, sizeof(class_label)) == 0 ||
                                get_json_string(single_obj_json, "class", class_label, sizeof(class_label)) == 0) { // Try 'class' if 'class_label' not found
                                int img_id = get_img_id_from_class_name(class_label);
                                if (img_id != -1) {
                                    send_target_result_to_android(context->android_fd, task_args->obstacle_id, img_id);
                                    printf("[ImgThread] Sent image detection result to Android: obstacle_id=%d, class_label=%s, img_id=%d\n", task_args->obstacle_id, class_label, img_id);
                                } else {
                                    fprintf(stderr, "[ImgThread] Unknown class label received: %s\n", class_label);
                                }
                            } else {
                                fprintf(stderr, "[ImgThread] Could not extract class label from image server response.\n");
                            }
                        }
                    }
                }
            } else {
                printf("[ImgThread] No object detected by image server for obstacle %d.\n", task_args->obstacle_id);
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

        } else {
            send_command_to_stm32(context->stm32_fd, cmd);
            sleep(1);
        }
    }
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
            // Default robot_x, robot_y, robot_dir as per dashboard.py and AlgorithmInput defaults
            snprintf(payload, sizeof(payload), "{\"obstacles\":[%s],\"robot_x\":1,\"robot_y\":1,\"robot_dir\":0,\"retrying\":false}", obstacles_str);
            printf("[NavThread] Pathfinding payload: %s\n", payload);

            char response[4096]; // Increased response buffer size
            if (post_data_to_server(PATHFINDING_SERVER_URL, payload, response, sizeof(response)) == 0) {
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
    char buffer[2048]; // Buffer for incoming Android messages

    while (1) {
        printf("[AndroidThread] Listening for messages...\n");
        // Blocking read until a message is received
        ssize_t bytes_read = read(context->android_fd, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            printf("[AndroidThread] Received: %s\n", buffer);

            // Android JSON messages are expected to be enclosed in quotes and end with newline
            // e.g., "START{\"obstacles\":[...]}"\n
            if (buffer[0] == '"' && bytes_read >= 2 && buffer[bytes_read-2] == '"') { // Check for quotes and newline
                // Remove outer quotes and newline for parsing
                char* json_start = buffer + 1;
                buffer[bytes_read-2] = '\0'; // Null-terminate before the last quote
                
                if (strncmp(json_start, "START", 5) == 0) {
                    pthread_mutex_lock(&context->lock);
                    if (context->state == STATE_IDLE) {
                        // Pass the JSON part (after "START") to the new parser
                        if (parse_android_map_and_obstacles(json_start + 5, context) == 0) {
                            context->new_map_received = true;
                            send_message_to_android_with_ack(context->android_fd, "\"Map received. Pathfinding...\"\n");
                            pthread_cond_signal(&context->new_task_cond);
                        } else {
                            send_message_to_android_with_ack(context->android_fd, "\"Error: Invalid map format.\"\n");
                        }
                    } else {
                         send_message_to_android_with_ack(context->android_fd, "\"Error: Robot is busy. Cannot start new mission.\"\n");
                    }
                    pthread_mutex_unlock(&context->lock);

                } else if (strncmp(json_start, "STOP", 4) == 0) {
                    pthread_mutex_lock(&context->lock);
                    send_message_to_android_with_ack(context->android_fd, "\"STOP command received.\"\n");
                    context->stop_requested = true;
                    if(context->state != STATE_IDLE) {
                        pthread_cond_signal(&context->new_task_cond);
                    }
                    pthread_mutex_unlock(&context->lock);
                } else {
                    fprintf(stderr, "[AndroidThread] Unrecognized command from Android: %s\n", json_start);
                }
            } else {
                fprintf(stderr, "[AndroidThread] Malformed message from Android (missing quotes or newline): %s\n", buffer);
            }
        } else if (bytes_read == 0) {
            printf("[AndroidThread] Read 0 bytes, serial port might be closed or empty.\n");
        } else {
            perror("[AndroidThread] Error reading from serial port");
            // Add a small delay to prevent busy-waiting on error
            usleep(100000); // 100ms
        }
    }
    return NULL;
}


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


    g_app_context.stm32_fd = init_serial_port(STM32_DEVICE, BAUD_RATE);
    g_app_context.android_fd = init_serial_port(ANDROID_DEVICE, BAUD_RATE);

    if (g_app_context.stm32_fd == -1 || g_app_context.android_fd == -1) {
        fprintf(stderr, "Fatal: Failed to initialize serial ports. Exiting.\n");
        return 1;
    }

    printf("--- RPi Control Centre Initialized ---\n");

    pthread_t android_tid, nav_tid;
    pthread_create(&android_tid, NULL, android_listener_thread, &g_app_context);
    pthread_create(&nav_tid, NULL, navigation_executor_thread, &g_app_context);

    pthread_join(android_tid, NULL);
    pthread_join(nav_tid, NULL);

    pthread_mutex_destroy(&g_app_context.lock);
    pthread_cond_destroy(&g_app_context.new_task_cond);
    close(g_app_context.stm32_fd);
    close(g_app_context.android_fd);

    curl_global_cleanup(); // Clean up curl once at application shutdown
    return 0;
}
