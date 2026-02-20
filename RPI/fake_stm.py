import os
import time
import re

RPI_TO_STM_PIPE = "rpi_to_stm"  # RPi -> FakeSTM
STM_TO_RPI_PIPE = "stm_to_rpi"  # FakeSTM -> RPi
ACK_DELAY_SECONDS = 1

def run_fake_stm():
    """
    Simulates an STM32 device using two separate named pipes for robust
    bidirectional communication, matching the C application's test harness.
    """
    print("Fake STM32: Starting.")

    # Ensure pipes exist before opening
    for pipe_name in [RPI_TO_STM_PIPE, STM_TO_RPI_PIPE]:
        if not os.path.exists(pipe_name):
            os.mkfifo(pipe_name)
            print(f"Fake STM32: Created named pipe '{pipe_name}'")

    try:
        # Open the command pipe for reading. This will block until the C program opens it for writing.
        print(f"Fake STM32: Opening '{RPI_TO_STM_PIPE}' for reading commands...")
        read_fd = os.open(RPI_TO_STM_PIPE, os.O_RDONLY)
        print("Fake STM32: Command pipe opened.")

        # Open the ACK pipe for writing. This will block until the C program opens it for reading.
        print(f"Fake STM32: Opening '{STM_TO_RPI_PIPE}' for writing ACKs...")
        write_fd = os.open(STM_TO_RPI_PIPE, os.O_WRONLY)
        print("Fake STM32: ACK pipe opened. Ready for communication.")

    except Exception as e:
        print(f"Fake STM32: Error opening pipes: {e}")
        print("Ensure the main C program is started and attempts to open both pipes.")
        return

    cmd_pattern = re.compile(rb":(\d+)/")
    read_buffer = b""

    try:
        while True:
            # Read raw data from the command pipe. This will block until data is available.
            chunk = os.read(read_fd, 4096)
            if not chunk:
                # An empty read indicates the other end of the pipe has closed.
                print("Fake STM32: Command pipe was closed by the RPi. Shutting down.")
                break
            
            read_buffer += chunk

            # Process all complete messages (ending in ';') in the buffer.
            while b';' in read_buffer:
                message, read_buffer = read_buffer.split(b';', 1)
                
                message_str = message.decode('utf-8', errors='ignore').strip()
                if not message_str:
                    continue

                print(f"Fake STM32: Received command: '{message_str};'")

                if message.startswith(b':'):
                    match = cmd_pattern.match(message)
                    if match:
                        cmd_id = int(match.group(1))
                        print(f"Fake STM32: Simulating processing for command ID {cmd_id}...")
                        time.sleep(ACK_DELAY_SECONDS)

                        ack_message = f"!{cmd_id}/DONE;\n".encode('utf-8')
                        
                        # Write the ACK to the dedicated ACK pipe
                        os.write(write_fd, ack_message)
                        print(f"Fake STM32: Sent ACK: '{ack_message.decode('utf-8').strip()}'")
                    else:
                        print(f"Fake STM32: Unrecognized command format: '{message_str};'")
                else:
                    print(f"Fake STM32: (Ignoring non-command data): '{message_str};'")
    except Exception as e:
        print(f"Fake STM32: An unexpected error occurred: {e}")
    finally:
        print("Fake STM32: Shutting down.")
        os.close(read_fd)
        os.close(write_fd)

if __name__ == "__main__":
    run_fake_stm()
