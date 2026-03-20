# Livestream

This livestream pipeline is for **Fastest Car**:

- **Pi**: capture camera frames + stream JPEG over TCP (no YOLO on Pi)
- **PC**: receive stream + run YOLO + keep a “stable” Left/Right Arrow candidate
- **Trigger**: Pi sends `LOCK 1` / `LOCK 2` to PC → PC saves `locked_1.jpg` / `locked_2.jpg` and sends `RESULT {json}` back to Pi on the stream socket

### On PC
Edit `Image/livestream/.env.pc`:

- `RPI_HOST=<PI_IP>` (example: `192.168.0.21`)
- `STREAM_PORT=5001` (keep default unless you changed it)

### On Pi
mkdir livestream
Create `.env.rpi` in the same folder as `stream_server.py`

## If you are using the C "brain" (STM + Android)
If you run `RPI/multithread_communication.c` (or the compiled `*_center` executables), you must set the PC IP used for LOCK triggers:

- In `RPI/multithread_communication.c`, update:
  - `PC_LOCK_HOST = "<YOUR_PC_IP>";` (example: `192.168.0.11`)

This lets the Pi trigger `LOCK 1` / `LOCK 2` to your PC automatically when STM requests a capture.

### 1) Pi: start the stream server
python3 -m pip install --user python-dotenv

In the Pi folder that contains `stream_server.py` and `.env.rpi`:

```bash
python3 stream_server.py
```

You should see:

- `Stream server listening on 0.0.0.0:5001`

### 2) PC: start YOLO client + trigger listener
In `Image/livestream/` on your PC:

```powershell
python stream_test.py
```

This will:

- connect to the Pi stream
- run YOLO on the PC
- listen for LOCK triggers on **port 5002** (default)

Saved images go to:

- `Image/live_detect/locked_1.jpg`
- `Image/live_detect/locked_2.jpg`

## WITHOUT STM
### 3) Trigger a lock (manual test from Pi)
From the Pi, send a trigger to your **PC IP** (replace `192.168.0.11` with your PC IP):

```bash
echo "LOCK 1" | nc -N <PC IP> 5002
echo "LOCK 2" | nc -N 192.168.0.11 5002
```

On the Pi you should see a log like:

- `RESULT payload: {... "img_id": 38/39, "object_id": "1"/"2" ...}`

## Image viewer (optional)

To view the saved `locked_*.jpg` files in a browser:

- Open `Image/livestream/pokemon.html` (it references `../live_detect/locked_1.jpg` and `locked_2.jpg`).

