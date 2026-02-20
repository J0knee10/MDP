#ifndef RPI_HAL_H
#define RPI_HAL_H

#include "shared_types.h"

// Struct to hold data for curl's WriteMemoryCallback
struct MemoryStruct {
  char *memory;
  size_t size;
};

/**
 * @file rpi_hal.h
 * @brief Hardware Abstraction Layer for RPi Robot Controller.
 *
 * Declares functions for interacting with hardware components and network services.
 * This version is more abstract and uses the shared data types.
 */

// --- Initialization ---
int init_serial_port(const char* device, int baud_rate);

// --- Android Communication ---
int send_status_to_android(int fd, const char* status);
// Changed from send_image_result_to_android to reflect "TARGET" command in Python
int send_target_result_to_android(int fd, int obstacle_id, int recognized_image_id);
// New function to parse the full Android JSON, including obstacles with 'd' and robot start position
int parse_android_map_and_obstacles(const char* json_string, SharedAppContext* context);
// New function to parse and execute direct STM commands from Android
int parse_and_execute_android_command(int stm32_fd, const char* android_command_str, SharedAppContext* context);
// New function for sending messages to Android with acknowledgment/retries
int send_message_to_android_with_ack(int fd, const char* message);
// New function to send standardized ACK messages to Android
int send_android_ack(int fd, const char* original_cat, const char* status_message);

// --- PC/Server Communication ---
int post_data_to_server(const char* url, const char* payload, char* response_buffer, int buffer_size);
// Modified to pass SharedAppContext to store snap_positions and robot initial position
int parse_command_route_from_server(const char* json_string, Command commands[], int* command_count, SnapPosition snap_positions[], int* snap_position_count);

// --- STM32 Communication ---
uint32_t send_command_to_stm32(int fd, Command command, uint32_t external_cmd_id);

// --- Camera/Image Processing ---
int capture_image(const char* filename);

int get_img_id_from_class_name(const char* class_name);

// --- Helper functions ---
size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp);


#endif // RPI_HAL_H
