import argparse
import os
import sys
import time
import cv2
import json
import socket
import threading
from collections import Counter, deque
from pathlib import Path

# Adjust this import to match your file name that defines StreamListener.
# e.g., if your class is in stream_client.py, use:
# from stream_client import StreamListener
from StreamListener import StreamListener


def main():
    parser = argparse.ArgumentParser(description="Test StreamListener with YOLO")
    default_weights = (Path(__file__).resolve().parents[1] / "models" / "best_20260211_210831.pt")
    default_save_dir = (Path(__file__).resolve().parents[1] / "live_detect")
    parser.add_argument(
        "--weights",
        default=str(default_weights),
        help=f"Path to YOLO weights (default: {default_weights})",
    )
    parser.add_argument("--conf", type=float, default=0.7, help="Confidence threshold")
    parser.add_argument("--infer-fps", type=float, default=8.0, help="YOLO inference rate (default: 8 FPS)")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference size (default: 320)")
    parser.add_argument(
        "--sync",
        action="store_true",
        help="Run YOLO on every frame (higher latency). Default is async throttled inference.",
    )
    parser.add_argument("--no-gui", default=False, help="Disable live window")
    parser.add_argument(
        "--save-dir",
        default=str(default_save_dir),
        help=f"Directory to save frames (default: {default_save_dir})",
    )
    parser.add_argument(
        "--save-mode",
        choices=["off", "detections", "all", "best", "stable"],
        default="stable",
        help=(
            "Saving behavior: "
            "off=don't save; "
            "detections=save only when a detection occurs; "
            "all=save every YOLO callback (includes no-detection frames); "
            "best=keep only best-confidence detection as best.jpg; "
            "stable=save only when a class is stable over time (fast false-positive filter)"
        ),
    )
    # "stable" mode tuning (defaults chosen for low delay at ~8 FPS)
    parser.add_argument("--stable-k", type=int, default=8, help="Stable mode window size K (default: 8)")
    parser.add_argument("--stable-m", type=int, default=5, help="Stable mode required votes M in window (default: 5)")
    parser.add_argument(
        "--stable-consec",
        type=int,
        default=3,
        help="Stable mode required consecutive wins (default: 3)", #change to 5 if we want more accurate
    )
    parser.add_argument(
        "--stable-min-conf",
        type=float,
        default=0.55,
        help="Stable mode minimum confidence per win (default: 0.55)",
    )
    parser.add_argument(
        "--stable-allow",
        default="Left Arrow,Right Arrow",
        help="Comma-separated labels allowed to lock (default: Left Arrow,Right Arrow)",
    )
    parser.add_argument(
        "--stable-resend-sec",
        type=float,
        default=0.70,
        help="In stable mode, resend locked result every N seconds while stable (default: 0.70)",
    )
    parser.add_argument(
        "--lock-mode",
        choices=["auto", "triggered"],
        default="triggered",
        help=(
            "auto=send RESULT whenever stable (older behavior); "
            "triggered=only send/save when Pi triggers LOCK <n> (recommended for 2 obstacles)"
        ),
    )
    parser.add_argument(
        "--lock-listen-host",
        default="0.0.0.0",
        help="PC host to listen for LOCK triggers (default: 0.0.0.0)",
    )
    parser.add_argument(
        "--lock-listen-port",
        type=int,
        default=5002,
        help="PC port to listen for LOCK triggers (default: 5002)",
    )
    args = parser.parse_args()

    if not Path(args.weights).exists():
        parser.error(
            f"weights file not found: {args.weights}\n"
            f"Either place the model at {default_weights} or pass --weights <path>."
        )

    save_dir = Path(args.save_dir) if args.save_mode != "off" else None
    if save_dir is not None:
        save_dir.mkdir(parents=True, exist_ok=True)

    listener = StreamListener(weights=args.weights)

    last_t = None
    frame_idx = 0
    best_conf = -1.0
    # Stable lock state (based on YOLO ticks, not raw camera frames)
    win_hist = deque(maxlen=max(1, int(args.stable_k)))
    last_win = None
    consec = 0
    locked_label = None
    last_sent_label = None
    last_sent_t = 0.0
    allow_set = {s.strip() for s in str(args.stable_allow).split(",") if s.strip()}
    # Map labels to the numeric img_id the RPi/STM expects
    LABEL_TO_IMG_ID = {
        "Left Arrow": 39,
        "Right Arrow": 38,
    }

    def post_plot_stm_ids(res, annotated):
        """Draw YOLO-style labels with ``<class> id=<stm_id> <conf>`` for mapped classes."""
        if res is None or len(res.boxes) == 0:
            return annotated
        try:
            from ultralytics.utils.plotting import colors
        except ImportError:
            def colors(cls_i, _):
                return (128, 128, 128)

        names = res.names
        for i in range(len(res.boxes)):
            cls_i = int(res.boxes.cls[i])
            name = str(names[cls_i])
            conf_f = float(res.boxes.conf[i])
            img_id = LABEL_TO_IMG_ID.get(name)
            label = f"{name} id={img_id} {conf_f:.2f}" if img_id is not None else f"{name} {conf_f:.2f}"
            box = res.boxes.xyxy[i].detach().cpu().numpy().reshape(-1)
            x1, y1 = int(box[0]), int(box[1])
            p1 = [x1, y1]
            col = colors(cls_i, True)
            sf, tf = 0.5, 1
            w, h = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, sf, tf)[0]
            h += 3
            outside = p1[1] >= h
            if p1[0] > annotated.shape[1] - w:
                p1[0] = annotated.shape[1] - w
            p2 = (p1[0] + w, p1[1] - h if outside else p1[1] + h)
            cv2.rectangle(annotated, p1, p2, col, -1, cv2.LINE_AA)
            cv2.putText(
                annotated,
                label,
                (p1[0], p1[1] - 2 if outside else p1[1] + h - 1),
                cv2.FONT_HERSHEY_SIMPLEX,
                sf,
                (255, 255, 255),
                tf,
                cv2.LINE_AA,
            )
        return annotated

    # Trigger queue: Pi connects to PC and sends "LOCK 1" / "LOCK 2"
    lock_requests = deque()
    lock_requests_lock = threading.Lock()

    def _lock_server():
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((args.lock_listen_host, int(args.lock_listen_port)))
            s.listen(5)
            print(f"[LOCK] Listening on {args.lock_listen_host}:{int(args.lock_listen_port)} (send: LOCK 1 / LOCK 2)")
            while True:
                conn, addr = s.accept()
                with conn:
                    try:
                        f = conn.makefile("rb")
                        for line in f:
                            cmd = line.decode("utf-8", "ignore").strip()
                            if not cmd:
                                continue
                            if cmd.upper().startswith("LOCK"):
                                parts = cmd.split()
                                lock_id = parts[1] if len(parts) >= 2 else "1"
                                with lock_requests_lock:
                                    lock_requests.append(lock_id)
                                try:
                                    conn.sendall(b"OK\n")
                                except Exception:
                                    pass
                                print(f"\n[LOCK] Trigger received: {cmd}")
                            else:
                                try:
                                    conn.sendall(b"ERR\n")
                                except Exception:
                                    pass
                    except Exception:
                        pass

    if args.save_mode == "stable" and args.lock_mode == "triggered":
        threading.Thread(target=_lock_server, daemon=True).start()

    last_stable = {
        "label": None,
        "conf": 0.0,
        "annotated": None,  # numpy image
        "votes": 0,
        "k": 0,
        "consec": 0,
        "t": 0.0,
    }

    def on_result(res, annotated_frame):
        nonlocal last_t, frame_idx, best_conf, last_win, consec, locked_label, last_sent_label, last_sent_t
        frame_idx += 1
        now = time.perf_counter()
        fps = 1.0 / (now - last_t) if last_t else 0.0
        last_t = now

        # Determine the "winning" detection for this YOLO tick (highest confidence)
        win_label = None
        win_conf = 0.0
        if res is not None:
            try:
                names = res.names
                cls_ids = res.boxes.cls.tolist() if hasattr(res.boxes.cls, "tolist") else res.boxes.cls
                confs = res.boxes.conf.tolist() if hasattr(res.boxes.conf, "tolist") else res.boxes.conf
                # Pick best by confidence
                best_i = None
                best_c = -1.0
                for i, c in enumerate(confs):
                    c = float(c)
                    if c > best_c:
                        best_c = c
                        best_i = i
                if best_i is not None:
                    win_label = str(names[int(cls_ids[best_i])])
                    win_conf = float(best_c)
            except Exception:
                win_label = None
                win_conf = 0.0

        if res is None:
            # No detections
            print(f"[{frame_idx:06d}] no detections | fps={fps:.1f}", end="\r")
        else:
            # Summarize detections
            names = res.names
            cls_ids = res.boxes.cls.tolist() if hasattr(res.boxes.cls, "tolist") else res.boxes.cls
            confs = res.boxes.conf.tolist() if hasattr(res.boxes.conf, "tolist") else res.boxes.conf
            dets = ", ".join(f"{names[int(c)]}:{float(conf):.2f}" for c, conf in zip(cls_ids, confs))

            if args.save_mode == "stable":
                # In stable mode we only care about allowed labels; reduce noise.
                if win_label is None:
                    print(f"[{frame_idx:06d}] det(s): {dets} | fps={fps:.1f} (ignored)", end="\r")
                elif win_label in allow_set:
                    print(f"[{frame_idx:06d}] det(s): {dets} | fps={fps:.1f}")
                else:
                    print(f"[{frame_idx:06d}] det(s): {dets} | fps={fps:.1f} (ignored)", end="\r")
            else:
                print(f"[{frame_idx:06d}] {len(cls_ids)} det(s): {dets} | fps={fps:.1f}   ")

        # Saving logic (runs for both detection and no-detection frames)
        if save_dir is None:
            return

        # Update stable state (only counts wins that clear min confidence)
        if args.save_mode == "stable":
            if win_label is not None and win_label in allow_set and win_conf >= float(args.stable_min_conf):
                win_hist.append(win_label)
                if win_label == last_win:
                    consec += 1
                else:
                    last_win = win_label
                    consec = 1
            else:
                # no valid win => break streak, but keep history as-is
                last_win = None
                consec = 0

            # Lock condition: enough consecutive + enough votes in window
            if consec >= int(args.stable_consec) and len(win_hist) >= int(args.stable_consec):
                counts = Counter(win_hist)
                top_label, top_votes = counts.most_common(1)[0]
                if top_votes >= int(args.stable_m) and top_label == last_win:
                    locked_label = top_label
                    now_t = time.perf_counter()
                    # Keep a continuously updated "stable candidate" (no delay on trigger)
                    last_stable["label"] = locked_label
                    last_stable["conf"] = float(win_conf)
                    last_stable["annotated"] = annotated_frame
                    last_stable["votes"] = int(top_votes)
                    last_stable["k"] = int(len(win_hist))
                    last_stable["consec"] = int(consec)
                    last_stable["t"] = now_t

                    if args.lock_mode == "auto":
                        resend_due = (now_t - last_sent_t) >= float(args.stable_resend_sec)
                        if locked_label != last_sent_label or resend_due:
                            lock_id = "auto"
                        else:
                            return
                    else:
                        # triggered: only send/save when we have a pending LOCK request
                        with lock_requests_lock:
                            lock_id = lock_requests.popleft() if lock_requests else None
                        if lock_id is None:
                            return

                    # Save + send exactly for this lock_id
                    img_id = LABEL_TO_IMG_ID.get(locked_label)
                    payload = {
                        "count": 1 if img_id is not None else 0,
                        "detected": img_id is not None,
                        "objects": [
                            {
                                "bbox": None,
                                "class": locked_label,
                                "class_id": locked_label,
                                "class_label": locked_label,
                                "confidence": float(win_conf),
                                "img_id": img_id,
                            }
                        ]
                        if img_id is not None
                        else [],
                        "success": img_id is not None,
                        "object_id": str(lock_id),
                    }

                    # Save snapshot (2 obstacles => locked_1.jpg / locked_2.jpg)
                    if annotated_frame is not None:
                        out_path = save_dir / f"locked_{lock_id}.jpg"
                        try:
                            cv2.imwrite(str(out_path), annotated_frame)
                            print(
                                f"\n[LOCKED {lock_id}] {locked_label} img_id={img_id} "
                                f"votes={top_votes}/{len(win_hist)} consec={consec} saved={out_path}"
                            )
                        except Exception as e:
                            print(f"\nFailed to save locked frame: {e}")
                    else:
                        print(
                            f"\n[LOCKED {lock_id}] {locked_label} img_id={img_id} votes={top_votes}/{len(win_hist)} consec={consec}"
                        )

                    listener.send_control("RESULT " + json.dumps(payload))
                    last_sent_label = locked_label
                    last_sent_t = now_t
            return

        should_save = (
            args.save_mode == "all"
            or (args.save_mode == "detections" and res is not None)
            or (args.save_mode == "best" and res is not None)
        )
        if not should_save:
            return

        try:
            if args.save_mode == "best":
                # best = max confidence in this result
                confs = res.boxes.conf.tolist() if hasattr(res.boxes.conf, "tolist") else res.boxes.conf
                conf_max = float(max(confs)) if confs is not None and len(confs) else 0.0
                if conf_max <= best_conf:
                    return
                best_conf = conf_max
                out_path = save_dir / "best.jpg"
            else:
                out_path = save_dir / f"frame_{frame_idx:06d}.jpg"

            frame_to_save = annotated_frame
            if frame_to_save is None:
                # In legacy callback mode, annotated_frame may be raw for no-detection frames.
                # If it is None, just skip saving.
                return
            cv2.imwrite(str(out_path), frame_to_save)
        except Exception as e:
            print(f"\nFailed to save frame: {e}")

    def on_disconnect():
        print("\nDisconnected from server.")

    try:
        listener.start_stream_read(
            on_result=on_result,
            on_disconnect=on_disconnect,
            conf_threshold=args.conf,
            show_video=not args.no_gui,
            async_infer=not args.sync,
            infer_fps=args.infer_fps,
            imgsz=args.imgsz,
            plot_kwargs={"labels": False},
            post_plot_fn=post_plot_stm_ids,
        )
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        listener.close()


if __name__ == "__main__":
    # Ensure we can import the client module when running from different working dirs
    sys.exit(main())