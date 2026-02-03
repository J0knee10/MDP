#ifndef RPI_HAL_H
#define RPI_HAL_H

#include "shared_types.h"

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
int send_image_result_to_android(int fd, int obstacle_id, int recognized_image_id);
int parse_obstacle_map_from_android(const char* json_string, Obstacle obstacles[], int* obstacle_count);

// --- PC/Server Communication ---
int post_data_to_server(const char* url, const char* payload, char* response_buffer, int buffer_size);
int parse_command_route_from_server(const char* json_string, Command commands[], int* command_count);

// --- STM32 Communication ---
int send_command_to_stm32(int fd, Command command);

// --- Camera/Image Processing ---
int capture_image(const char* filename);

// --- Helper functions ---
size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp);

#endif // RPI_HAL_H
