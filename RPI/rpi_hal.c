#include "rpi_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <curl/curl.h>

/**
 * @file rpi_hal.c
 * @brief Implements the Hardware Abstraction Layer for the RPi Robot Controller.
 */

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

// Struct for libcurl to write response data into a buffer.
struct MemoryStruct {
    char *memory;
    size_t size;
};

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
}

// --- Android Communication ---

int send_status_to_android(int fd, const char* status) {
    char buffer[256];
    // Format: {"type": "status", "value": "message"}\n
    snprintf(buffer, sizeof(buffer), "{\"type\": \"status\", \"value\": \"%s\"}\n", status);
    return write_to_serial(fd, buffer);
}

int send_image_result_to_android(int fd, int obstacle_id, int recognized_image_id) {
    char buffer[256];
    // Format: {"type": "image_result", "obstacle_id": 1, "image_id": 11}\n
    snprintf(buffer, sizeof(buffer), "{\"type\": \"image_result\", \"obstacle_id\": %d, \"image_id\": %d}\n", obstacle_id, recognized_image_id);
    return write_to_serial(fd, buffer);
}

int parse_obstacle_map_from_android(const char* json_string, Obstacle obstacles[], int* obstacle_count) {
    // Expects a simple JSON format: {"obstacles": [{"id":1,"x":10,"y":5}, ...]}
    *obstacle_count = 0;
    char* temp_str = strdup(json_string); // Make a copy to modify with strtok

    char* token = strtok(temp_str, "{}[],:\" "); // Characters to split by
    while(token != NULL && *obstacle_count < MAX_OBSTACLES) {
        if (strcmp(token, "id") == 0) {
            obstacles[*obstacle_count].id = atoi(strtok(NULL, "{}[],:\" "));
        } else if (strcmp(token, "x") == 0) {
            obstacles[*obstacle_count].x = atoi(strtok(NULL, "{}[],:\" "));
        } else if (strcmp(token, "y") == 0) {
            obstacles[*obstacle_count].y = atoi(strtok(NULL, "{}[],:\" "));
            (*obstacle_count)++; // Increment after finding the last member of the struct
        }
        token = strtok(NULL, "{}[],:\" ");
    }
    free(temp_str);
    return (*obstacle_count > 0) ? 0 : -1;
}


// --- PC/Server Communication ---

int post_data_to_server(const char* url, const char* payload, char* response_buffer, int buffer_size) {
    CURL* curl;
    CURLcode res;
    int result = -1;

    struct MemoryStruct chunk;
    chunk.memory = malloc(1);
    chunk.size = 0;

    curl_global_init(CURL_GLOBAL_ALL);
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
            }
        }
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);
    }
    free(chunk.memory);
    curl_global_cleanup();
    return result;
}

int parse_command_route_from_server(const char* json_string, Command commands[], int* command_count) {
    // Expects: {"route": [{"type":"FW", "value":10}, {"type":"TR"}, ...]}
    *command_count = 0;
    char* temp_str = strdup(json_string); // Copy for strtok

    char* token = strtok(temp_str, "{}[],:\" ");
    while (token != NULL && *command_count < MAX_COMMANDS) {
        if (strcmp(token, "type") == 0) {
            char* type_str = strtok(NULL, "{}[],:\" ");
            if (strcmp(type_str, "FW") == 0) commands[*command_count].type = CMD_MOVE_FORWARD;
            else if (strcmp(type_str, "TL") == 0) commands[*command_count].type = CMD_TURN_LEFT;
            else if (strcmp(type_str, "TR") == 0) commands[*command_count].type = CMD_TURN_RIGHT;
            else if (strcmp(type_str, "SS") == 0) commands[*command_count].type = CMD_SNAPSHOT;
        } else if (strcmp(token, "value") == 0) {
            commands[*command_count].value = atoi(strtok(NULL, "{}[],:\" "));
            (*command_count)++; // Finished a command
        }
        token = strtok(NULL, "{}[],:\" ");
    }

    free(temp_str);
    return (*command_count > 0) ? 0 : -1;
}

// --- STM32 Communication ---

int send_command_to_stm32(int fd, Command command) {
    char stm_command[64];
    const int MOVE_SPEED_PWM = 5000; // A default PWM value for forward movement
    const int TURN_SPEED_PWM = 4000; // A default PWM value for turning

    switch (command.type) {
        case CMD_MOVE_FORWARD:
            // Format based on STM32 code: "FWD,speed,distance;"
            snprintf(stm_command, sizeof(stm_command), "FWD,%d,%d;", MOVE_SPEED_PWM, command.value);
            break;
        case CMD_TURN_LEFT:
            // Format based on STM32 code: "TURNL,speed,angle;"
            snprintf(stm_command, sizeof(stm_command), "TURNL,%d,%d;", TURN_SPEED_PWM, command.value);
            break;
        case CMD_TURN_RIGHT:
            // Format based on STM32 code: "TURNR,speed,angle;"
            snprintf(stm_command, sizeof(stm_command), "TURNR,%d,%d;", TURN_SPEED_PWM, command.value);
            break;
        case CMD_SNAPSHOT:
            // Snapshots are handled by the RPi itself and are not sent to the STM32.
            return 0;
        default:
            fprintf(stderr, "send_command_to_stm32: Unknown command type\n");
            return -1; // Unknown command
    }
    printf("[To STM32]: %s\n", stm_command);

    // write_to_serial sends the exact string, which is what the STM32 expects (terminated by ';')
    return write_to_serial(fd, stm_command);
}

// --- Camera/Image Processing ---

int capture_image(const char* filename) {
    char command[256];
    // Use raspistill for Buster OS compatibility. Arguments are slightly different.
    snprintf(command, sizeof(command), "raspistill -n -t 200 -w 640 -h 480 -o %s", filename);
    int result = system(command);
    if (result == 0) {
        printf("[Camera] Image captured: %s\n", filename);
    } else {
        fprintf(stderr, "[Camera] Failed to capture image.\n");
    }
    return result;
}

