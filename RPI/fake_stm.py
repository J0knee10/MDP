import os
import time
import re
import select # For non-blocking read with timeout
import errno # For specific error checking

RPI_TO_STM_PIPE = "rpi_to_stm"
ACK_DELAY_SECONDS = 1  # Simulate STM32 processing time (e.g., 1s)
WRITE_RETRY_DELAY = 0.05 # Small delay before retrying a blocked write

def run_fake_stm():
    print(f"Fake STM32: Starting. Listening on '{RPI_TO_STM_PIPE}'")

    try:
        # Open the pipe in read-write mode, and non-blocking.
        # This allows select.select to manage reading without blocking indefinitely,
        # and makes write operations raise BlockingIOError if the buffer is full.
        pipe_fd = os.open(RPI_TO_STM_PIPE, os.O_RDWR | os.O_NONBLOCK)
                                                     
    except OSError as e:
        print(f"Fake STM32: Error opening pipe '{RPI_TO_STM_PIPE}': {e}")
        print("Please ensure the pipe 'rpi_to_stm' exists and the RPi program is running.")
        print("Make sure the RPi program ('./test_center') is started *before* 'fake_stm.py' or vice-versa,")
        print("and both are trying to open the pipe.")
        return

    # Regex to extract command ID from commands like ":ID/MOTOR/FWD/PWM/VAL;"
    cmd_pattern = re.compile(rb"^:(\d+)/") 
    
    read_buffer = b"" # Use bytes for direct os.read/write
    while True:
        # Use select to wait for data to become available on the pipe_fd for reading.
        # This prevents busy-waiting and allows a graceful shutdown if the other end closes.
        # It also checks writeable status before attempting to write.
        rlist, wlist, _ = select.select([pipe_fd], [pipe_fd], [], 0.1) # Check every 100ms
        
        # --- Handle Read Operations ---
        if pipe_fd in rlist: 
            try:
                chunk = os.read(pipe_fd, 4096) 
                if not chunk: # Pipe closed by other end (EOF)
                    print("Fake STM32: RPi end closed the pipe. Shutting down.")
                    break
                
                read_buffer += chunk
                
                # Process messages terminated by ';'
                while b';' in read_buffer:
                    message_bytes, _, _ = read_buffer.partition(b';') 
                    read_buffer = read_buffer[len(message_bytes) + len(b';'):]
                    
                    message_str = message_bytes.decode('utf-8', errors='ignore').strip()
                    
                    if message_str.startswith(':'):
                        print(f"Fake STM32: Received command: '{message_str};'")
                        match = cmd_pattern.match(message_bytes.strip()) 
                        if match:
                            cmd_id = int(match.group(1).decode('utf-8')) 
                            print(f"Fake STM32: Simulating processing for command ID {cmd_id}...")
                            time.sleep(ACK_DELAY_SECONDS) 

                            ack_message = f"!{cmd_id}/DONE;\n".encode('utf-8')
                            
                            # --- Handle Write Operations (ACK sending) ---
                            written_bytes = 0
                            while written_bytes < len(ack_message):
                                try:
                                    # Ensure the pipe is writeable before attempting os.write
                                    if pipe_fd in wlist: 
                                        bytes_just_written = os.write(pipe_fd, ack_message[written_bytes:])
                                        written_bytes += bytes_just_written
                                    else: # Not writeable, wait and re-select
                                        print(f"Fake STM32: Write buffer full for ACK {cmd_id}. Retrying...")
                                        time.sleep(WRITE_RETRY_DELAY)
                                        # Re-select to update wlist
                                        rlist, wlist, _ = select.select([pipe_fd], [pipe_fd], [], 0.1) 
                                        # If still not writeable after delay, loop will check again
                                        continue 
                                except BlockingIOError:
                                    print(f"Fake STM32: Write operation blocked for ACK {cmd_id}. Retrying...")
                                    time.sleep(WRITE_RETRY_DELAY)
                                    rlist, wlist, _ = select.select([pipe_fd], [pipe_fd], [], 0.1) 
                                    continue
                                except Exception as write_e:
                                    print(f"Fake STM32: Error during ACK write: {write_e}")
                                    break 

                            print(f"Fake STM32: Sent ACK: '{ack_message.decode('utf-8').strip()}'")
                        else:
                            print(f"Fake STM32: Unrecognized command format: '{message_str};'")
                    elif message_str.startswith('!'):
                        print(f"Fake STM32: (Ignoring echoed ACK from RPi): '{message_str};'")
                    else:
                        print(f"Fake STM32: (Ignoring unknown pipe data): '{message_str};'")
            except BlockingIOError:
                # If os.read() raises BlockingIOError here despite select,
                # it just means no data is currently available.
                # Simply continue the loop to re-check.
                pass 
            except Exception as e:
                print(f"Fake STM32: An unexpected error occurred during read/process: {e}")
                break
        # If no data available for read, just loop and check again after select timeout
        else: 
            pass
            
    os.close(pipe_fd)
    print("Fake STM32: Shutting down.")

if __name__ == "__main__":
    run_fake_stm()