#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#define ANDROID_PIPE_PATH "android_to_rpi"

int main() {
    int fd;
    // The mission to send. This can be customized.
    const char* mission_command = "START{\"obstacles\":[{\"id\":1,\"x\":10,\"y\":5}, {\"id\":2,\"x\":20,\"y\":30}, {\"id\":3,\"x\":5,\"y\":25}]}";

    printf("[Fake Android Client] Attempting to open named pipe: %s\n", ANDROID_PIPE_PATH);

    // Open the named pipe for writing. This will block until the reader (multithread_communication.c) opens it.
    fd = open(ANDROID_PIPE_PATH, O_WRONLY);
    if (fd == -1) {
        perror("[Fake Android Client] Failed to open named pipe");
        fprintf(stderr, "Ensure 'mkfifo %s' has been run and the main RPi application is running in RPI_TESTING mode.\n", ANDROID_PIPE_PATH);
        return 1;
    }

    printf("[Fake Android Client] Named pipe opened successfully. Sending mission command:\n%s\n", mission_command);

    // Write the mission command to the pipe
    ssize_t bytes_written = write(fd, mission_command, strlen(mission_command));
    if (bytes_written == -1) {
        perror("[Fake Android Client] Failed to write to named pipe");
        close(fd);
        return 1;
    } else if (bytes_written != strlen(mission_command)) {
        fprintf(stderr, "[Fake Android Client] Warning: Incomplete write to named pipe.\n");
    }

    printf("[Fake Android Client] Mission command sent. Closing pipe.\n");

    close(fd);
    return 0;
}
