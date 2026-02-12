#include "rpi_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <errno.h>
#include <curl/curl.h>
#include <time.h> // For usleep in send_message_to_android_with_ack

#include "json_parser.h" // New include for JSON parsing helpers

/**
 * @file rpi_hal.c
 * @brief Implements the Hardware Abstraction Layer for the RPi Robot Controller.
 */

// --- Configuration for Android Communication ---
#define ANDROID_COMM_MAX_RETRIES 3
#define ANDROID_COMM_RETRY_DELAY_US 300000 // 300ms

// --- Mappings from Python task1.py ---
// These are static and internal to rpi_hal.c

// Maps class names to image IDs
static const struct {
    const char* class_name;
    int img_id;
} IMAGE_MAPPING_C[] = {
    {"Number 1", 11}, {"Number 2", 12}, {"Number 3", 13}, {"Number 4", 14},
    {"Number 5", 15}, {"Number 6", 16}, {"Number 7", 17}, {"Number 8", 18},
    {"Number 9", 19}, {"Alphabet A", 20}, {"Alphabet B", 21}, {"Alphabet C", 22},
    {"Alphabet D", 23}, {"Alphabet E", 24}, {"Alphabet F", 25}, {"Alphabet G", 26},
    {"Alphabet H", 27}, {"Alphabet S", 28}, {"Alphabet T", 29}, {"Alphabet U", 30},
    {"Alphabet V", 31}, {"Alphabet W", 32}, {"Alphabet X", 33}, {"Alphabet Y", 34},
    {"Alphabet Z", 35}, {"Up Arrow", 36}, {"Down Arrow", 37}, {"Right Arrow", 38},
    {"Left Arrow", 39}, {"Stop sign", 40}
};
static const size_t IMAGE_MAPPING_COUNT = sizeof(IMAGE_MAPPING_C) / sizeof(IMAGE_MAPPING_C[0]);

// Function to map class name string to image ID
int get_img_id_from_class_name(const char* class_name) {
    for (size_t i = 0; i < IMAGE_MAPPING_COUNT; i++) {
        if (strcmp(IMAGE_MAPPING_C[i].class_name, class_name) == 0) {
            return IMAGE_MAPPING_C[i].img_id;
        }
    }
    return -1; // Not found
}


// --- Internal Helper Functions ---

// Helper to write a string to a serial port.
static int write_to_serial(int fd, const char* message) {
    ssize_t bytes_written = write(fd, message, strlen(message));
    if (bytes_written < 0) {
        perror("write_to_serial: Failed to write");
        return -1;
    }
    return 0;
}

// Callback for libcurl to write data from a response.
size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp) {
    size_t realsize = size * nmemb;
    struct MemoryStruct *mem = (struct MemoryStruct *)userp;

    char *ptr = realloc(mem->memory, mem->size + realsize + 1);
    if(ptr == NULL) {
        printf("not enough memory (realloc returned NULL)\n");
        return 0;
    }

    mem->memory = ptr;
    memcpy(&(mem->memory[mem->size]), contents, realsize);
    mem->size += realsize;
    mem->memory[mem->size] = 0;

    return realsize;
}

// --- Public API Implementation ---

int init_serial_port(const char* device, int baud_rate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("init_serial_port: Unable to open device");
        return -1;
    }

#ifdef RPI_TESTING
    // For named pipes, termios settings are not applicable.
    // We just need the file descriptor.
    fcntl(fd, F_SETFL, 0); // Ensure blocking write for named pipes
    printf("Named pipe %s opened successfully for testing.\n", device);
    return fd;
#else
    // For real serial ports, apply termios settings
    fcntl(fd, F_SETFL, 0); // Set to blocking mode

    struct termios options;
    tcgetattr(fd, &options);

    speed_t speed;
    switch (baud_rate) {
        case 9600:   speed = B9600;   break;
        case 115200: speed = B115200; break;
        default:     fprintf(stderr, "Unsupported baud rate\n"); close(fd); return -1;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_lflag |= ICANON; // Use canonical mode to read line by line
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    printf("Serial port %s initialized successfully.\n", device);
    return fd;
#endif
}

// --- Android Communication ---

int send_status_to_android(int fd, const char* status) {
    char buffer[256];
    // Format: {"type": "status", "value": "\"message\""}\n (message itself is quoted)
    snprintf(buffer, sizeof(buffer), "{\"type\": \"status\", \"value\": \"%s\"}\n", status);
    return write_to_serial(fd, buffer);
}

// New function: Sends a message to Android with retries (mimics Python's send_with_ack)
int send_message_to_android_with_ack(int fd, const char* message) {
    for (int attempt = 0; attempt < ANDROID_COMM_MAX_RETRIES; attempt++) {
        printf("[AndroidComm] Attempt %d: Sending %s", attempt + 1, message);
        if (write_to_serial(fd, message) == 0) {
            // For now, we assume success after writing.
            // A full ACK mechanism would involve reading from 'fd' for a response.
            return 0; // Success
        }
        usleep(ANDROID_COMM_RETRY_DELAY_US); // Delay before retry
    }
    fprintf(stderr, "[AndroidComm] Failed to send message after %d attempts: %s", ANDROID_COMM_MAX_RETRIES, message);
    return -1; // Failure
}


// New function: send_target_result_to_android (replaces old send_image_result_to_android)
int send_target_result_to_android(int fd, int obstacle_id, int recognized_image_id) {
    char buffer[256];
    char json_buffer[512]; // Buffer to hold the full JSON string
    // Python format: resp = "TARGET," + str(object_id) + "," + str(class_name)
    // Then json.dumps(resp) + "\n"
    snprintf(buffer, sizeof(buffer), "TARGET,%d,%d", obstacle_id, recognized_image_id);
    snprintf(json_buffer, sizeof(json_buffer), "\"%s\"\n", buffer); // Enclose in quotes and add newline

    return send_message_to_android_with_ack(fd, json_buffer);
}

// New function: parse_android_map_and_obstacles (replaces old parse_obstacle_map_from_android)
int parse_android_map_and_obstacles(const char* json_string, SharedAppContext* context) {
    return parse_android_map_json(json_string, context);
}


// --- PC/Server Communication ---

int post_data_to_server(const char* url, const char* payload, char* response_buffer, int buffer_size) {
    CURL* curl;
    CURLcode res;
    int result = -1;

    struct MemoryStruct chunk;
    chunk.memory = malloc(1);
    chunk.size = 0;

    // curl_global_init(CURL_GLOBAL_ALL); // This is now done once in main
    curl = curl_easy_init();
    if (curl) {
        struct curl_slist *headers = NULL;
        headers = curl_slist_append(headers, "Content-Type: application/json");

        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 20L);

        res = curl_easy_perform(curl);
        if (res != CURLE_OK) {
            fprintf(stderr, "post_data_to_server failed: %s\n", curl_easy_strerror(res));
        } else {
            long response_code;
            curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &response_code);
            if (response_code >= 200 && response_code < 300) {
                strncpy(response_buffer, chunk.memory, buffer_size - 1);
                response_buffer[buffer_size - 1] = '\0';
                result = 0; // Success
            } else {
                fprintf(stderr, "post_data_to_server received non-2xx response: %ld\n", response_code);
            }
        }
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    } else {
        fprintf(stderr, "post_data_to_server: curl_easy_init() failed.\n");
    }
    free(chunk.memory);
    // curl_global_cleanup(); // This is now done once in main
    return result;
}

// Modified parse_command_route_from_server
int parse_command_route_from_server(const char* json_string, Command commands[], int* command_count, SnapPosition snap_positions[], int* snap_position_count) {
    return parse_route_json(json_string, commands, command_count, snap_positions, snap_position_count);
}

// --- STM32 Communication ---

int send_command_to_stm32(int fd, Command command, uint32_t external_cmd_id) {
    char stm_command[128];
    static uint32_t internal_cmd_id_counter = 0; // Static to maintain ID across calls
    const int DEFAULT_MOVE_SPEED_PERCENTAGE = 70; // 70% speed
    const int DEFAULT_TURN_SPEED_PERCENTAGE = 60; // 60% speed

    uint32_t cmd_id_to_use;
    if (external_cmd_id != 0) {
        cmd_id_to_use = external_cmd_id;
    } else {
        internal_cmd_id_counter++;
        cmd_id_to_use = internal_cmd_id_counter;
    }

    switch (command.type) {
        case CMD_MOVE_FORWARD:
            // STM32 format: :<cmdid>/MOTOR/FWD/<param1Speed>/<param2DistAngle>;
            snprintf(stm_command, sizeof(stm_command), ":%u/MOTOR/FWD/%d/%d;",
                     cmd_id_to_use, DEFAULT_MOVE_SPEED_PERCENTAGE, command.value);
            break;
        case CMD_MOVE_BACKWARD: // Added for BW command
            // STM32 format: :<cmdid>/MOTOR/BWD/<param1Speed>/<param2DistAngle>;
            snprintf(stm_command, sizeof(stm_command), ":%u/MOTOR/BWD/%d/%d;",
                     cmd_id_to_use, DEFAULT_MOVE_SPEED_PERCENTAGE, command.value);
            break;
        case CMD_TURN_LEFT:
            // STM32 format: :<cmdid>/MOTOR/TURNL/<param1Speed>/<param2DistAngle>;
            snprintf(stm_command, sizeof(stm_command), ":%u/MOTOR/TURNL/%d/%d;",
                     cmd_id_to_use, DEFAULT_TURN_SPEED_PERCENTAGE, command.value);
            break;
        case CMD_TURN_RIGHT:
            // STM32 format: :<cmdid>/MOTOR/TURNR/<param1Speed>/<param2DistAngle>;
            snprintf(stm_command, sizeof(stm_command), ":%u/MOTOR/TURNR/%d/%d;",
                     cmd_id_to_use, DEFAULT_TURN_SPEED_PERCENTAGE, command.value);
            break;
        case CMD_SNAPSHOT:
            printf("[To STM32]: Skipping snapshot command (handled by RPi).\n");
            return 0; // Indicate success but no command sent to STM32
        default:
            fprintf(stderr, "send_command_to_stm32: Unknown command type (%d)\n", command.type);
            return -1; // Unknown command
    }
    printf("[To STM32]: %s\n", stm_command); // Add newline for clear logging, STM32 expects ';' as terminator

    return write_to_serial(fd, stm_command);
}

// --- Camera/Image Processing ---

int capture_image(const char* filename) {
#ifdef RPI_TESTING
    printf("[Camera] (TEST MODE) Faking image capture: %s\n", filename);
    // Create a dummy file for testing purposes
    FILE* fp = fopen(filename, "w");
    if (fp) {
        // Write a minimal valid JPEG header (or just some dummy content)
        // This is a very simplified placeholder. A real dummy JPEG would be larger.
        // For testing the *flow*, an empty file or a small dummy is usually enough.
        fprintf(fp, "Fake JPEG content for %s", filename);
        fclose(fp);
        return 0; // Success
    } else {
        fprintf(stderr, "[Camera] (TEST MODE) Failed to create dummy image file.\n");
        return -1; // Failure
    }
#else
    char command[256];
    // Use raspistill for Buster OS compatibility. Arguments are slightly different.
    // -n: No preview
    // -t 200: Take picture after 200ms delay (gives camera time to adjust)
    // -w 640 -h 480: Set resolution
    // -o: Output file
    // Added -q 75 for quality to reduce file size slightly, though not critical for function
    snprintf(command, sizeof(command), "raspistill -n -t 200 -w 640 -h 480 -q 75 -o %s", filename);
    printf("[Camera] Executing command: %s\n", command);
    int result = system(command);
    if (result == 0) {
        printf("[Camera] Image captured: %s\n", filename);
    } else {
        fprintf(stderr, "[Camera] Failed to capture image. Error code: %d\n", result);
    }
    return result;
#endif
}