#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <curl/curl.h>
#include <time.h>
#include <errno.h>
#include <ctype.h>
#include <signal.h>
#include <fcntl.h>
#include <termios.h>

#include "shared_types.h"
#include "rpi_hal.h"
#include "json_parser.h"

// --- Configuration ---
const char* STM32_HW_DEVICE = "/dev/ttyACM0";
const char* IMAGE_SERVER_URL = "http://192.168.22.21:4000/detect";
const int BAUD_RATE = 115200;
const char* CAPTURE_FILENAME = "task2_capture.jpg";

// --- Global Variables ---
int g_stm32_fd = -1;
volatile int g_keep_running = 1;
pthread_mutex_t g_stm32_mutex = PTHREAD_MUTEX_INITIALIZER;

// --- Signal Handler ---
void handle_sigint(int sig) {
    printf("\n[Signal] SIGINT (Ctrl+C) received. Exiting...\n");
    g_keep_running = 0;
}

// --- Helper: Send STM Command ---
void send_stm_command(const char* command, int p1, int p2) {
    char stm_command[128];
    static uint32_t cmd_id = 200; // Distinct ID range for Task 2
    
    snprintf(stm_command, sizeof(stm_command), ":%u/MOTOR/%s/%d/%d;", cmd_id++, command, p1, p2);
    
    pthread_mutex_lock(&g_stm32_mutex);
    write(g_stm32_fd, stm_command, strlen(stm_command));
    tcdrain(g_stm32_fd);
    pthread_mutex_unlock(&g_stm32_mutex);
    
    printf("[To STM]: %s\n", stm_command);
}

// --- Image Server Posting (Robust Version) ---
static int post_image_to_server(int obstacle_id, char* response_buffer, int buffer_size) {
    CURL* curl;
    CURLcode res;
    int result = -1;

    struct MemoryStruct chunk = { .memory = malloc(1), .size = 0 };
    if (chunk.memory == NULL) {
        fprintf(stderr, "[Img] Failed to allocate memory for CURL response.\n");
        return -1;
    }

    curl = curl_easy_init();
    if (curl) {
        curl_mime *form = curl_mime_init(curl);
        curl_mimepart *field;

        // Image file part
        field = curl_mime_addpart(form);
        curl_mime_name(field, "image");
        curl_mime_filedata(field, CAPTURE_FILENAME);
        
        // Object ID part
        char id_str[10];
        snprintf(id_str, sizeof(id_str), "%d", obstacle_id);
        field = curl_mime_addpart(form);
        curl_mime_name(field, "object_id");
        curl_mime_data(field, id_str, CURL_ZERO_TERMINATED);

        curl_easy_setopt(curl, CURLOPT_URL, IMAGE_SERVER_URL);
        curl_easy_setopt(curl, CURLOPT_MIMEPOST, form);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 15L); // 15s timeout for Task 2 speed

        res = curl_easy_perform(curl);
        if (res == CURLE_OK) {
            long code;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &code);
            if (code >= 200 && code < 300) {
                if (response_buffer != NULL && chunk.memory != NULL) {
                    strncpy(response_buffer, chunk.memory, buffer_size - 1);
                    response_buffer[buffer_size - 1] = '\0';
                }
                result = 0;
            } else {
                fprintf(stderr, "[Img] Server error: %ld\n", code);
            }
        } else {
            fprintf(stderr, "[Img] CURL failed: %s\n", curl_easy_strerror(res));
        }
        curl_easy_cleanup(curl);
        curl_mime_free(form);
    }
    
    free(chunk.memory);
    return result;
}

// --- Thread Logic for Image Capture & Recognition ---
typedef struct {
    int capture_id; // 1 or 2
} CaptureTaskArgs;

void* process_capture_thread(void* args) {
    CaptureTaskArgs* task_args = (CaptureTaskArgs*)args;
    int cid = task_args->capture_id;
    char response[2048];
    int recognized_dir = 0; // 0=None, 1=Left, 2=Right

    printf("[Task2] Starting capture %d processing...\n", cid);
    
    if (capture_image(CAPTURE_FILENAME) != 0) {
        fprintf(stderr, "[Task2] Image capture failed for CAPTURE%d.\n", cid);
    } else {
        if (post_image_to_server(cid, response, sizeof(response)) == 0) {
            printf("[Task2] Server Response: %s\n", response);
            
            int count = 0;
            if (get_json_int(response, "count", &count) == 0 && count > 0) {
                // Robust parsing of "objects" array to find arrows
                // We'll use the img_id from class_label or img_id fields
                // Left Arrow = 39, Right Arrow = 38
                
                if (strstr(response, "\"img_id\":39") || strstr(response, "Left Arrow")) {
                    recognized_dir = 1;
                } else if (strstr(response, "\"img_id\":38") || strstr(response, "Right Arrow")) {
                    recognized_dir = 2;
                }
            }
        }
    }

    if (recognized_dir > 0) {
        printf("[Task2] RECOGNIZED CAPTURE%d: %s\n", cid, recognized_dir == 1 ? "LEFT" : "RIGHT");
        char cmd_name[20];
        snprintf(cmd_name, sizeof(cmd_name), "CAPTURE%d", cid);
        send_stm_command(cmd_name, recognized_dir, 0);
    } else {
        printf("[Task2] No arrow recognized for CAPTURE%d. Retrying capture request...\n", cid);
        // Depending on STM32 logic, we might need to send a 'retry' or just wait for next request
        // For now, we don't send anything, letting STM32 timeout or user intervene if needed.
        // Or we could send a special 'None' value if the STM32 handles it.
    }

    free(task_args);
    return NULL;
}

int main() {
    signal(SIGINT, handle_sigint);
    setvbuf(stdout, NULL, _IONBF, 0);
    curl_global_init(CURL_GLOBAL_ALL);

    printf("--- RPi Task 2 Control Start ---\n");

    g_stm32_fd = init_serial_port(STM32_HW_DEVICE, BAUD_RATE);
    if (g_stm32_fd == -1) {
        fprintf(stderr, "Fatal: Could not open STM32 port %s\n", STM32_HW_DEVICE);
        return 1;
    }

    // Clear stale data
    flush_serial_port(g_stm32_fd);

    // Optional: Send initial TASK2 command to start STM FSM if not already started
    // send_stm_command("TASK2", 0, 0);

    char buffer[256];
    char main_buffer[512];
    int main_buffer_pos = 0;

    printf("[Task2] Listening for STM32 requests...\n");

    while (g_keep_running) {
        ssize_t bytes_read = read(g_stm32_fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            
            if (main_buffer_pos + bytes_read < sizeof(main_buffer) - 1) {
                memcpy(main_buffer + main_buffer_pos, buffer, bytes_read);
                main_buffer_pos += bytes_read;
                main_buffer[main_buffer_pos] = '\0';
            } else {
                fprintf(stderr, "[Task2] Buffer overflow! Resetting.\n");
                main_buffer_pos = 0;
            }

            char* semicolon;
            while ((semicolon = strchr(main_buffer, ';')) != NULL) {
                size_t msg_len = (semicolon - main_buffer) + 1;
                char msg[256];
                strncpy(msg, main_buffer, msg_len);
                msg[msg_len] = '\0';
                
                printf("[From STM]: %s\n", msg);

                // Handle Image Capture Requests
                if (strstr(msg, "!CAPTURE1")) {
                    printf("[Task2] Received CAPTURE1 request.\n");
                    CaptureTaskArgs* args = malloc(sizeof(CaptureTaskArgs));
                    args->capture_id = 1;
                    pthread_t tid;
                    pthread_create(&tid, NULL, process_capture_thread, args);
                    pthread_detach(tid);
                } 
                else if (strstr(msg, "!CAPTURE2")) {
                    printf("[Task2] Received CAPTURE2 request.\n");
                    CaptureTaskArgs* args = malloc(sizeof(CaptureTaskArgs));
                    args->capture_id = 2;
                    pthread_t tid;
                    pthread_create(&tid, NULL, process_capture_thread, args);
                    pthread_detach(tid);
                }
                else if (strstr(msg, "/DONE")) {
                    printf("[Task2] STM32 signalled TASK2 DONE! Exiting.\n");
                    g_keep_running = 0;
                }

                // Shift remaining data to front
                int remaining = main_buffer_pos - msg_len;
                if (remaining > 0) {
                    memmove(main_buffer, main_buffer + msg_len, remaining);
                    main_buffer_pos = remaining;
                } else {
                    main_buffer_pos = 0;
                }
                main_buffer[main_buffer_pos] = '\0';
            }
        } else if (bytes_read < 0 && errno != EAGAIN) {
            perror("read");
            break;
        } else {
            usleep(10000); // 10ms sleep
        }
    }

    close(g_stm32_fd);
    curl_global_cleanup();
    printf("--- RPi Task 2 Control End ---\n");
    return 0;
}
