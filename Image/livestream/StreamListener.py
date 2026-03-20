import socket
import struct
import cv2
import numpy as np
import os
import inspect  
import threading
import time

from dotenv import load_dotenv
from pathlib import Path

# Prefer a PC-specific env if present; fall back to .env.
_ENV_DIR = Path(__file__).resolve().parent
load_dotenv(dotenv_path=_ENV_DIR / ".env.pc")
load_dotenv(dotenv_path=_ENV_DIR / ".env")

from ultralytics import YOLO


class StreamListener:
    """
    TCP client for Pi streaming server.
    Protocol:
      - send 'stream_request\\n'
      - server replies with a single line (e.g., 'OK STREAMING\\n')
      - then a loop of: [4-byte big-endian length][JPEG bytes]
      - to stop: send 'STOP\\n' and close

    Callback signatures supported:
      (A) on_result(result, annotated_frame, raw_frame)
      (B) on_result(result, annotated_frame)   # legacy
          - when there are no detections, annotated_frame will be the raw frame (not None)
    """
    def __init__(self, weights):
        self.HOST = os.getenv("RPI_HOST")
        # socket.create_connection expects int port
        self.PORT = int(os.getenv("STREAM_PORT", "0"))
        self.REQ_STREAM = bytes(os.getenv("REQ_STREAM") + "\n", "utf-8")
        self.STOP_STREAM = bytes(os.getenv("STOP_STREAM") + "\n", "utf-8")
        self.PING_STREAM = bytes(os.getenv("PING_STREAM") + "\n", "utf-8")

        self.model = YOLO(weights)
        self.sock = None

    # --- low-level helpers ---
    def _connect(self):
        self.sock = socket.create_connection((self.HOST, self.PORT), timeout=5)
        try:
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass
        self.sock.settimeout(None)

    def _readline(self, maxlen=256):
        buf = bytearray()
        while len(buf) < maxlen:
            ch = self.sock.recv(1)
            if not ch:
                return None
            buf += ch
            if ch == b"\n":
                break
        return bytes(buf)

    def _recv_exact(self, n):
        data = bytearray()
        while len(data) < n:
            chunk = self.sock.recv(n - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)

    # --- NEW: smart callback invoker (3-arg or 2-arg) ---
    def _invoke_on_result(self, cb, res, annotated, raw):
        if cb is None:
            return
        try:
            sig = inspect.signature(cb)
            params = len(sig.parameters)
        except Exception:
            # Fallback: try 3-arg, else 2-arg
            params = 3
        try:
            if params >= 3:
                cb(res, annotated, raw)
            else:
                # legacy 2-arg: if no detections, send the raw frame instead of None
                cb(res, annotated if annotated is not None else raw)
        except Exception as e:
            print(f"[StreamListener] on_result error: {e}")

    def req_stream(self):
        if self.sock is None:
            self._connect()
        self.sock.sendall(self.REQ_STREAM)
        header = self._readline()
        print(header.decode("utf-8") if header else "NO HEADER")

    def send_control(self, line: str):
        """
        Send a line-delimited control message to the Pi over the same TCP socket.
        Safe to call while streaming (Pi reads commands in a separate thread).
        """
        if not self.sock:
            return
        try:
            data = (line.strip() + "\n").encode("utf-8")
            self.sock.sendall(data)
        except Exception:
            pass

    def start_stream_read(
        self,
        on_result,
        on_disconnect,
        conf_threshold=0.7,
        show_video=True,
        *,
        async_infer=True,
        infer_fps=8.0,
        imgsz=416,
        post_plot_fn=None,
        plot_kwargs=None,
    ):
        """
        Connects, requests stream, and reads frames.

        For low-latency display, set async_infer=True (default):
          - the main loop shows the latest raw frame immediately
          - a background thread runs YOLO at ~infer_fps on the newest frame
          - callbacks fire on YOLO results (not every frame)

        If async_infer=False, YOLO runs on every frame (higher latency).

        Calls:
          - on_result(result, annotated_frame, raw_frame)  # preferred
          - or on_result(result, annotated_frame)          # legacy; raw is used if annotated is None
          - on_disconnect() when the stream ends
        Press ESC to stop (if show_video=True).

        post_plot_fn: optional callable ``(result, annotated_bgr) -> ndarray`` invoked after
        ``res.plot()`` so callers can draw extra overlays (e.g. custom IDs) on the live window.
        plot_kwargs: optional dict of keyword arguments forwarded to ``Results.plot()`` (e.g.
        ``{"labels": False}`` to hide default labels so ``post_plot_fn`` can draw its own).
        """
        stop_evt = threading.Event()
        _plot_kw = plot_kwargs if isinstance(plot_kwargs, dict) else {}

        # Shared state between reader and inference thread
        state_lock = threading.Lock()
        latest_raw = {"frame": None, "t": 0.0}
        latest_vis = {"frame": None, "t": 0.0}  # annotated or last detection frame
        new_frame_evt = threading.Event()

        def _infer_loop():
            min_dt = 1.0 / max(float(infer_fps), 0.1)
            next_t = 0.0
            while not stop_evt.is_set():
                # wait for at least one frame, but wake periodically to check stop_evt
                new_frame_evt.wait(0.25)
                new_frame_evt.clear()
                if stop_evt.is_set():
                    break

                now = time.perf_counter()
                if now < next_t:
                    continue

                with state_lock:
                    frame = latest_raw["frame"]
                    if frame is None:
                        continue
                    # copy so main thread can keep updating latest_raw
                    frame_for_infer = frame.copy()

                try:
                    res = self.model.predict(
                        frame_for_infer,
                        save=False,
                        imgsz=imgsz,
                        conf=conf_threshold,
                        verbose=False,
                    )[0]
                except Exception:
                    # If inference fails, skip this tick
                    next_t = time.perf_counter() + min_dt
                    continue

                if len(res.boxes) > 0:
                    annotated = res.plot(**_plot_kw)
                    if post_plot_fn is not None:
                        try:
                            annotated = post_plot_fn(res, annotated)
                        except Exception as e:
                            print(f"[StreamListener] post_plot_fn error: {e}")
                    with state_lock:
                        latest_vis["frame"] = annotated
                        latest_vis["t"] = time.perf_counter()
                    self._invoke_on_result(on_result, res, annotated, frame_for_infer)
                else:
                    # No detections: still provide raw frame to callback consumers
                    with state_lock:
                        latest_vis["frame"] = frame_for_infer
                        latest_vis["t"] = time.perf_counter()
                    self._invoke_on_result(on_result, None, None, frame_for_infer)

                next_t = time.perf_counter() + min_dt

        try:
            self.req_stream()

            infer_thread = None
            if async_infer:
                infer_thread = threading.Thread(target=_infer_loop, daemon=True)
                infer_thread.start()

            while True:
                # 1) read 4-byte length header
                hdr = self._recv_exact(4)
                if hdr is None:
                    break
                size = struct.unpack("!I", hdr)[0]
                if size <= 0 or size > 50_000_000:  # sanity check (50MB cap)
                    break

                # 2) read JPEG payload
                jpg = self._recv_exact(size)
                if jpg is None:
                    break

                # 3) decode frame
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    # corrupted frame; skip but still keep UI responsive
                    self._invoke_on_result(on_result, None, None, None)
                    continue

                if async_infer:
                    with state_lock:
                        latest_raw["frame"] = frame
                        latest_raw["t"] = time.perf_counter()
                    new_frame_evt.set()

                    # show latest visualization if available, else raw
                    with state_lock:
                        disp = latest_vis["frame"] if latest_vis["frame"] is not None else frame
                else:
                    # synchronous: YOLO every frame (higher latency)
                    res = self.model.predict(
                        frame,
                        save=False,
                        imgsz=imgsz,
                        conf=conf_threshold,
                        verbose=False,
                    )[0]

                    if len(res.boxes) > 0:
                        annotated = res.plot(**_plot_kw)
                        if post_plot_fn is not None:
                            try:
                                annotated = post_plot_fn(res, annotated)
                            except Exception as e:
                                print(f"[StreamListener] post_plot_fn error: {e}")
                        self._invoke_on_result(on_result, res, annotated, frame)
                        disp = annotated
                    else:
                        self._invoke_on_result(on_result, None, None, frame)
                        disp = frame

                if show_video:
                    cv2.imshow("Stream", disp)
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:  # ESC
                        try:
                            self.sock.sendall(self.STOP_STREAM)
                        except Exception:
                            pass
                        break

        finally:
            stop_evt.set()
            new_frame_evt.set()
            # ensure cleanup
            try:
                if self.sock:
                    self.sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                if self.sock:
                    self.sock.close()
            except Exception:
                pass
            self.sock = None
            if show_video:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass
            if on_disconnect:
                on_disconnect()

    def close(self):
        try:
            if self.sock:
                try:
                    self.sock.sendall(self.STOP_STREAM)
                except Exception:
                    pass
                self.sock.close()
        finally:
            self.sock = None
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass