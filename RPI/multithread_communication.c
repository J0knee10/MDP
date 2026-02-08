#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <curl/curl.h>

#include "shared_types.h"
#include "rpi_hal.h"

// --- Configuration ---
#ifdef RPI_TESTING
const char* STM32_DEVICE = "rpi_to_stm";
const char* ANDROID_DEVICE = "android_to_rpi";
const char* PATHFINDING_SERVER_URL = "http://127.0.0.1:4000/path";
const char* IMAGE_SERVER_URL = "http://127.0.0.1:5000/detect";
#elif defined(FAKE_ANDROID_SIMULATION)
const char* STM32_DEVICE = "/dev/ttyACM0";
const char* ANDROID_DEVICE = "android_to_rpi";
const char* PATHFINDING_SERVER_URL = "http://127.0.0.1:4000/path";
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
static int post_image_to_server_thread(int obstacle_id, char* response_buffer, int buffer_size) {
    CURL* curl;
    CURLcode res;
    int result = -1;

    struct MemoryStruct { char *memory; size_t size; } chunk = { .memory = malloc(1), .size = 0 };
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

    printf("[ImgThread] Capturing image for obstacle %d...\n", task_args->obstacle_id);
    if (capture_image(CAPTURE_FILENAME) != 0) {
        fprintf(stderr, "[ImgThread] Failed to capture image.\n");
    } else {
        // The PC will handle displaying the processed image.
        if (post_image_to_server_thread(task_args->obstacle_id, NULL, 0) == 0) {
            printf("[ImgThread] ACK received from image server for obstacle %d.\n", task_args->obstacle_id);
        } else {
            fprintf(stderr, "[ImgThread] Failed to upload image or no ACK received.\n");
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
            args->context = context;
            args->obstacle_id = cmd.value;

            pthread_create(&tid, NULL, process_image_thread, args);
            pthread_detach(tid);

        } else {
            send_command_to_stm32(context->stm32_fd, cmd);
            sleep(1);
        }
    }
    send_status_to_android(context->android_fd, "Navigation complete.");
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
            char payload[2048] = "{\"obstacles\":[";
            for (int i = 0; i < context->obstacle_count; i++) {
                char obs_str[100];
                snprintf(obs_str, sizeof(obs_str), "{\"id\":%d,\"x\":%d,\"y\":%d}", context->obstacles[i].id, context->obstacles[i].x, context->obstacles[i].y);
                strcat(payload, obs_str); if (i < context->obstacle_count - 1) strcat(payload, ",");
            }
            strcat(payload, "]}");

            char response[2048];
            if (post_data_to_server(PATHFINDING_SERVER_URL, payload, response, sizeof(response)) == 0 &&
                parse_command_route_from_server(response, context->commands, &context->command_count) == 0) {
                send_status_to_android(context->android_fd, "Route calculated. Navigating.");
                execute_navigation();
            } else {
                send_status_to_android(context->android_fd, "Error: Pathfinding failed.");
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
    char buffer[2048];

    while (1) {
        printf("[AndroidThread] Listening for messages...\n");
        int bytes_read = read(context->android_fd, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            printf("[AndroidThread] Received: %s\n", buffer);

            if (strncmp(buffer, "START", 5) == 0) {
                pthread_mutex_lock(&context->lock);
                if (context->state == STATE_IDLE) {
                    if (parse_obstacle_map_from_android(buffer, context->obstacles, &context->obstacle_count) == 0) {
                        context->new_map_received = true;
                        send_status_to_android(context->android_fd, "Map received. Pathfinding...");
                        pthread_cond_signal(&context->new_task_cond);
                    } else {
                        send_status_to_android(context->android_fd, "Error: Invalid map format.");
                    }
                } else {
                     send_status_to_android(context->android_fd, "Error: Robot is busy. Cannot start new mission.");
                }
                pthread_mutex_unlock(&context->lock);

            } else if (strncmp(buffer, "STOP", 4) == 0) {
                pthread_mutex_lock(&context->lock);
                send_status_to_android(context->android_fd, "STOP command received.");
                context->stop_requested = true;
                if(context->state != STATE_IDLE) {
                    pthread_cond_signal(&context->new_task_cond);
                }
                pthread_mutex_unlock(&context->lock);
            }
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

/**
 * =================================================================================
 * HOW TO COMPILE AND RUN ON RASPBERRY PI (DEBIAN BUSTER OS)
 * =================================================================================
 *
 * This guide provides all the necessary steps to set up the environment,
 * compile the code, and run the application.
 *
 *
 * --- STEP 1: Install Dependencies ---
 *
 *   sudo apt-get update
 *   sudo apt-get install libcurl4-openssl-dev bluetooth bluez python3 libraspicam-dev
 *
 *
 * --- STEP 2: Configure Bluetooth (for live runs) ---
 *
 * 1. Start the bluetooth command-line tool: `bluetoothctl`
 * 2. Inside bluetoothctl, enter: `agent on`, `default-agent`, `scan on`
 * 3. Find your Android device's MAC address (e.g., AA:BB:CC:DD:EE:FF).
 * 4. Pair and trust it: `pair AA:BB:CC:DD:EE:FF`, `trust AA:BB:CC:DD:EE:FF`
 * 5. Exit bluetoothctl: `exit`
 * 6. Bind the device to /dev/rfcomm0: `sudo rfcomm bind 0 AA:BB:CC:DD:EE:FF 1`
 *
 *
 * --- STEP 3: Compile the Application ---
 *
 *   gcc -o control_center multithread_communication.c rpi_hal.c -I. -pthread -lcurl -Wall
 *
 *
 * --- STEP 4: Run the Application ---
 *
 *   Ensure you have permissions for serial ports, either by using `sudo` or by
 *   adding your user to the `dialout` group (`sudo usermod -a -G dialout $USER`
 *   then log out and back in).
 *
 *   sudo ./control_center
 *
 *
 * =================================================================================
 * HOW TO TEST THE APPLICATION (WITHOUT HARDWARE)
 * =================================================================================
 *
 * This procedure allows you to test the full application logic on your RPi
 * by faking the hardware components. You will need 4 terminal windows.
 *
 * --- In Terminal 1: Setup Fake Hardware Pipes ---
 *
 * Create two "named pipes". These are special files that let us redirect input
 * and output between processes.
 *
 *   mkfifo android_to_rpi
 *   mkfifo rpi_to_stm
 *
 *   # Now, listen on the fake STM pipe. This terminal will display commands
 *   # as the RPi application sends them.
 *   echo "--- Fake STM32 listening for commands... ---"
 *   cat < rpi_to_stm
 *
 *
 * --- In Terminal 2: Start Fake PC Servers ---
 *
 *   # In one command, start both Python fake servers in the background.
 *   python3 fake_path_server.py & python3 fake_image_server.py
 *
 *
 * --- In Terminal 3: Compile and Run the RPi App in Test Mode ---
 *
 *   # Compile the code with the RPI_TESTING flag.
 *   # This tells the code to use the named pipes instead of real serial ports.
 *   gcc -o test_center -DRPI_TESTING multithread_communication.c rpi_hal.c -I. -pthread -lcurl -Wall
 *   gcc -o STtest_center -DFAKE_ANDROID_SIMULATION multithread_communication.c rpi_hal.c -I. -pthread -lcurl -Wall   
 *   # Run the test version. It will start and wait for a command from the fake Android pipe.
 *   ./test_center
 *   ./STtest_center
 *
 *
 * --- In Terminal 4: Send Commands as Fake Android ---
 *
 *   # Wait until Terminal 3 shows "[AndroidThread] Listening...".
 *   # Then, send a START command with a map to begin the mission.
 *   echo 'START{"obstacles":[{"id":1,"x":10,"y":5}, {"id":2,"x":20,"y":30}, {"id":3,"x":5,"y":25}]}' > android_to_rpi
 *
 *
 * --- VERIFYING THE TEST ---
 *
 * 1.  **Logic Test:** As soon as you run the `echo` command in Terminal 4:
 *     - Terminal 3 (RPi App) should log that it received the map and is pathfinding.
 *     - Terminal 2 (Servers) should log that it received a path request.
 *     - Terminal 1 (Fake STM) should immediately start printing the robot commands:
 *       `FWD,5000,10;`
 *       `TURNR,4000,90;`
 *       ...etc.
 *
 * 2.  **Concurrency Test (Efficiency):**
 *     - When Terminal 1 prints a movement command that was *after* a snapshot, you
 *       should see NO 5-second delay. The robot moves on instantly.
 *     - A few seconds later, Terminal 3 (RPi App) will log that the image result
 *       was received and sent to Android.
 *     - This proves the image processing happened in the background without blocking navigation.
 *
 * 3.  **Concurrency Test (Responsiveness):**
 *     - While commands are streaming into Terminal 1, go to Terminal 4 and run:
 *       `echo 'STOP' > android_to_rpi`
 *     - The stream of commands in Terminal 1 should stop almost instantly, and the
 *       RPi app in Terminal 3 should log that it is aborting and returning to Idle.
 *       This proves the app is always responsive to high-priority commands.
 *
 */