import socket
import threading
import os
import time
import io
import struct
import logging
logging.basicConfig(level=logging.INFO)

from dotenv import load_dotenv
from pathlib import Path

# Prefer an RPi-specific env if present; fall back to .env.
_ENV_DIR = Path(__file__).resolve().parent
load_dotenv(dotenv_path=_ENV_DIR / ".env.rpi")
load_dotenv(dotenv_path=_ENV_DIR / ".env")

import picamera
import json

class StreamServer():
    def __init__(self):
        self.BUFFER_SIZE = int(os.getenv("BUFFER_SIZE"))
        self.HOST = os.getenv("RPI_HOST")
        self.PORT = int(os.getenv("STREAM_PORT"))
        self.REQ_STREAM = os.getenv("REQ_STREAM")
        self.STOP_STREAM = os.getenv("STOP_STREAM")
        self.PING_STREAM = os.getenv("PING_STREAM")
        
        self.server_socket = None
        self.client_socket = None
        
        self._server_stop = threading.Event()
        self._client_lock = threading.Lock()

        # Forward RESULT payloads to the C "brain" over localhost UDP
        # (so the C process remains the only writer to the STM serial port).
        self.RESULT_FWD_HOST = os.getenv("RESULT_FWD_HOST", "127.0.0.1")
        self.RESULT_FWD_PORT = int(os.getenv("RESULT_FWD_PORT", "5555"))

    def _forward_result_to_brain(self, payload: dict):
        try:
            msg = json.dumps(payload).encode("utf-8")
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.sendto(msg, (self.RESULT_FWD_HOST, self.RESULT_FWD_PORT))
        except Exception as e:
            logging.warning(f"Failed to forward RESULT to brain: {e}")
    
    def connect(self):
        """Accept clients and hand them off to a handler thread."""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # we mostly SEND, so tune the send buffer (recv buf doesn't matter much here)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, self.BUFFER_SIZE)

        self.server_socket.bind((self.HOST, self.PORT))
        self.server_socket.listen(1)
        logging.info(f"Stream server listening on {self.HOST}:{self.PORT}")

        try:
            while not self._server_stop.is_set():
                cs, addr = self.server_socket.accept()
                logging.info(f"Stream client connected: {addr}")

                with self._client_lock:
                    # If you want only one active client, close the previous one
                    if self.client_socket:
                        try:
                            self.client_socket.shutdown(socket.SHUT_RDWR)
                        except Exception:
                            pass
                        try:
                            self.client_socket.close()
                        except Exception:
                            pass
                    self.client_socket = cs

                th = threading.Thread(target=self._handle_client, args=(cs, addr), daemon=True)
                th.start()
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def _handle_client(self, cs: socket.socket, addr):
        """
        Handle one client:
        - control thread reads line-delimited commands
        - streaming loop sends length-prefixed JPEG frames
        - STOP or EOF => end session (close camera & socket)
        """
        # Per-client events
        session_alive = threading.Event()
        session_alive.set()
        streaming_on = threading.Event()  # becomes True after REQ_STREAM

        def _recv_commands():
            try:
                f = cs.makefile("rb")
                while session_alive.is_set():
                    line = f.readline()
                    if not line:
                        # EOF: client closed
                        break
                    cmd = line.strip().decode("utf-8", "ignore")
                    if cmd == self.REQ_STREAM:
                        streaming_on.set()
                        try: cs.sendall(b"OK STREAMING\n")
                        except Exception: pass
                    elif cmd == self.STOP_STREAM:
                        try: cs.sendall(b"OK STOPPING\n")
                        except Exception: pass
                        break  # end session
                    elif cmd == self.PING_STREAM:
                        try: cs.sendall(b"PONG\n")
                        except Exception: pass
                    elif cmd.startswith("RESULT "):
                        # RESULT <json> (from laptop): used to forward img_id to STM.
                        payload_str = cmd[len("RESULT "):].strip()
                        try:
                            payload = json.loads(payload_str)
                            logging.info(f"RESULT payload: {payload}")
                            self._forward_result_to_brain(payload)
                        except Exception as e:
                            logging.warning(f"Bad RESULT payload: {e}")
                    else:
                        # During streaming, do NOT send text replies on the same socket
                        # as the binary JPEG stream (it will corrupt the framing).
                        pass
            except Exception:
                # socket likely went away; treat as EOF
                pass
            finally:
                session_alive.clear()
                try: f.close()
                except Exception: pass

        # Start control thread
        try:
            cs.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            cs.settimeout(None)
        except Exception:
            pass
        ctrl = threading.Thread(target=_recv_commands, daemon=True)
        ctrl.start()

        try:
            with picamera.PiCamera(resolution=(640, 480), framerate=20) as cam:
                cam.rotation = 180
                time.sleep(0.2)
                stream = io.BytesIO()

                for _ in cam.capture_continuous(stream, format="jpeg", quality=45, use_video_port=True):
                    if not session_alive.is_set():
                        break  # client said STOP or disconnected

                    if not streaming_on.is_set():
                        # Not streaming yet (waiting for REQ_STREAM); idle briefly
                        stream.seek(0); stream.truncate()
                        time.sleep(0.02)
                        continue

                    size = stream.tell()
                    stream.seek(0)
                    try:
                        cs.sendall(struct.pack("!I", size))
                        cs.sendall(stream.read(size))
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        # Client gone in the middle of a send
                        break
                    finally:
                        stream.seek(0); stream.truncate()
        finally:
            # Clean up this client
            session_alive.clear()
            streaming_on.clear()
            try:
                cs.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                cs.close()
            except Exception:
                pass
            with self._client_lock:
                if self.client_socket is cs:
                    self.client_socket = None
            print(f"Client {addr} closed.")

    def close(self):
        self._server_stop.set()
        try:
            if self.client_socket:
                self.client_socket.close()
        except Exception:
            pass
        try:
            if self.server_socket:
                self.server_socket.close()
        except Exception:
            pass
        

if __name__ == "__main__":
    srv = StreamServer()
    try:
        srv.connect()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        srv.close()
