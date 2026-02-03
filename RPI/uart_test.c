#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "rpi_hal.h" // We will reuse the HAL functions

#define ANDROID_PIPE_PATH "android_to_rpi"

/**
 * @file uart_test.c
 * @brief A test utility to verify the message forwarding path.
 *
 * This program waits for a mission from a named pipe (simulating Android),
 * processes it into a hardcoded command list (simulating pathfinding),
 * and sends the commands to the physical STM32 serial port.
 */
int main() {
    printf("--- Starting Message-Forwarding UART Test ---\n");

    // --- Part 1: Wait for mission from Fake Android Client ---
    printf("Creating named pipe '%s' if it doesn't exist.\n", ANDROID_PIPE_PATH);
    mkfifo(ANDROID_PIPE_PATH, 0666); // Create the pipe, doesn't harm if it exists

    printf("Waiting for a mission command from the fake_android_client...\n");
    int android_fd = open(ANDROID_PIPE_PATH, O_RDONLY);
    if (android_fd == -1) {
        perror("Failed to open android pipe for reading");
        return 1;
    }

    char buffer[2048];
    int bytes_read = read(android_fd, buffer, sizeof(buffer) - 1);
    close(android_fd); // We are done with the pipe

    if (bytes_read <= 0) {
        fprintf(stderr, "Failed to read a command from the pipe.\n");
        return 1;
    }
    buffer[bytes_read] = '\0';
    printf("Received mission command: %s\n\n", buffer);

    // At this point, you could parse the buffer, but for this test,
    // we'll just use the reception of a message as the trigger.

    // --- Part 2: Initialize Physical UART and Send Commands ---
    printf("--- Mission Received. Initializing Physical UART ---\n");
    int stm32_fd = init_serial_port(STM32_DEVICE, BAUD_RATE);

    if (stm32_fd == -1) {
        fprintf(stderr, "Fatal: Failed to initialize STM32 serial port '%s'.\n", STM32_DEVICE);
        fprintf(stderr, "Check that the device is connected and you have the correct permissions (try running with sudo).\n");
        return 1;
    }

    printf("Serial port %s opened successfully. Sending commands...\n", STM32_DEVICE);
    printf("Watch your PC's serial monitor.\n\n");

    // A hardcoded route, similar to what the pathfinding server would return.
    // This simulates the "processing" step.
    Command route_to_send[] = {
        {.type = CMD_MOVE_FORWARD, .value = 200},
        {.type = CMD_TURN_LEFT,    .value = 45},
        {.type = CMD_MOVE_FORWARD, .value = 150},
        {.type = CMD_TURN_RIGHT,   .value = 90},
        {.type = CMD_MOVE_FORWARD, .value = 100}
    };
    int num_commands = sizeof(route_to_send) / sizeof(route_to_send[0]);

    // Loop through the hardcoded route and send each command
    for (int i = 0; i < num_commands; i++) {
        Command cmd_to_send = route_to_send[i];

        printf("Sending command %d/%d to STM port... ", i + 1, num_commands);
        if (send_command_to_stm32(stm32_fd, cmd_to_send) == 0) {
            printf("OK.\n");
        } else {
            fprintf(stderr, "Failed.\n");
        }
        sleep(1); // Pause for a second between commands
    }

    printf("\n--- All commands sent. Test complete. ---\n");
    close(stm32_fd);

    return 0;
}
