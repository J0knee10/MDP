"""
MDP IMAGE DETECTION SERVER
==========================

HOW TO RUN:
1. Navigate to this directory in your terminal:
   cd Image

2. Use the project virtual environment (recommended after git pull):
   Windows:  venv\Scripts\activate
   Mac/Linux: source venv/bin/activate

3. Install dependencies (First time only, or if you get ModuleNotFoundError):
   pip install -r requirements.txt

4. Start the server:
   python object_detection_server.py

5. Use the API:
   - Endpoint: POST http://<YOUR_IP>:4000/detect
   - Requires: 'image' (file) and 'object_id' (string) fields.

6. View detections in your browser:
   - Go to: http://localhost:4000/
   - This automatically runs display.py to refresh the gallery.

7. Test the /detect API with a picture (server must be running):
   - python test_detect.py path/to/your/image.jpg [object_id]
   - Or with curl: curl -X POST -F "image=@path/to/image.jpg" -F "object_id=1" http://localhost:4000/detect

NOTE: Ensure your YOLO model is in the './models' folder.
"""

import os
import sys
import time

import cv2
import numpy as np
from flask import Flask, jsonify, request, send_from_directory
from ultralytics import YOLO

app = Flask(__name__)

MODEL_PATH = "./models/best_20260211_210831.pt"
SAVE_DIRECTORY = "./detections"
CONFIDENCE_THRESHOLD = 0.5

# Request counter (1-based, resets when server restarts)
detect_request_count = 0

# Create save directory if it doesn't exist
os.makedirs(SAVE_DIRECTORY, exist_ok=True)

# Load YOLO model
print("Loading YOLO model...")
model = YOLO(MODEL_PATH)
print("New Model Classes:", model.names)
print("Model loaded successfully!")

# Map raw class names (e.g. "11", "12") to display labels. Used for plotting and API response.
CLASS_NAME_REMAP = {
    "11": "Number 1",
    "12": "Number 2",
    "13": "Number 3",
    "14": "Number 4",
    "15": "Number 5",
    "16": "Number 6",
    "17": "Number 7",
    "18": "Number 8",
    "19": "Number 9",
    "20": "Alphabet A",
    "21": "Alphabet B",
    "22": "Alphabet C",
    "23": "Alphabet D",
    "24": "Alphabet E",
    "25": "Alphabet F",
    "26": "Alphabet G",
    "27": "Alphabet H",
    "28": "Alphabet S",
    "29": "Alphabet T",
    "30": "Alphabet U",
    "31": "Alphabet V",
    "32": "Alphabet W",
    "33": "Alphabet X",
    "34": "Alphabet Y",
    "35": "Alphabet Z",
    "36": "Up Arrow",
    "37": "Down Arrow",
    "38": "Right Arrow",
    "39": "Left Arrow",
    "40": "Stop sign",
    "45": "Bullseye",
}

# Numeric IDs for Android/RPI; used as img_id in API response.
IMAGE_MAPPING = {
    "Number 1": 11, "Number 2": 12, "Number 3": 13, "Number 4": 14, "Number 5": 15,
    "Number 6": 16, "Number 7": 17, "Number 8": 18, "Number 9": 19,
    "Alphabet A": 20, "Alphabet B": 21, "Alphabet C": 22, "Alphabet D": 23,
    "Alphabet E": 24, "Alphabet F": 25, "Alphabet G": 26, "Alphabet H": 27,
    "Alphabet S": 28, "Alphabet T": 29, "Alphabet U": 30, "Alphabet V": 31,
    "Alphabet W": 32, "Alphabet X": 33, "Alphabet Y": 34, "Alphabet Z": 35,
    "Up Arrow": 36, "Down Arrow": 37, "Right Arrow": 38, "Left Arrow": 39,
    "Stop sign": 40,
    "Bullseye": 41,
}


def save_image_with_detections(image, results, filename):
    """Save image with bounding boxes drawn."""
    os.makedirs(SAVE_DIRECTORY, exist_ok=True)
    # Override labels used in plotting
    names_for_plot = {}
    for class_id, raw_name in results[0].names.items():
        raw_class_name = str(raw_name)
        class_label = CLASS_NAME_REMAP.get(raw_class_name, raw_class_name)
        if class_label != raw_class_name:
            names_for_plot[class_id] = f"{class_label} - {raw_class_name}"
        else:
            names_for_plot[class_id] = class_label
    results[0].names = names_for_plot
    annotated_image = results[0].plot()
    filepath = os.path.join(SAVE_DIRECTORY, filename)
    cv2.imwrite(filepath, annotated_image)
    print(f"Saved detection image to: {filepath}")

    return filepath


def save_original_image(image, filename):
    """Save original image without annotations."""
    filepath = os.path.join(SAVE_DIRECTORY, f"original_{filename}")
    os.makedirs(SAVE_DIRECTORY, exist_ok=True)
    cv2.imwrite(filepath, image)
    print(f"Saved original image to: {filepath}")
    return filepath


@app.route("/detect", methods=["POST"])
def detect_objects():
    """Receive image, run YOLO detection, return detections JSON."""
    try:
        if "image" not in request.files:
            return jsonify({"success": False, "error": "No image provided"}), 400

        file = request.files["image"]
        object_id = request.form.get("object_id")

        # Read image from request
        image_bytes = file.read()
        nparr = np.frombuffer(image_bytes, np.uint8)
        image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if image is None:
            return (
                jsonify({"success": False, "error": "Invalid image format"}),
                400,
            )

        global detect_request_count
        detect_request_count += 1
        req_num = detect_request_count
        print()
        h, w = image.shape[:2]
        dims = f"{h}x{w}"

        # Run YOLO detection
        t0 = time.perf_counter()
        results = model(image, conf=CONFIDENCE_THRESHOLD, verbose=False)
        elapsed_ms = (time.perf_counter() - t0) * 1000

        # Get detections
        detections = results[0].boxes
        num_detections = len(detections)
        filename = f"detection_{object_id}.jpg"

        # Build detection list for response
        detected_objects = []
        for box in detections:
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            raw_class_name = str(model.names[class_id])
            class_label = CLASS_NAME_REMAP.get(raw_class_name, raw_class_name)
            if class_label != raw_class_name:
                class_name = f"{class_label} - {raw_class_name}"
            else:
                class_name = class_label

            img_id = IMAGE_MAPPING.get(class_label, None)
            detected_objects.append(
                {
                    "class": class_name,
                    "class_label": class_label,
                    "class_id": raw_class_name,
                    "img_id": img_id,
                    "confidence": confidence,
                    "bbox": box.xyxy[0].tolist(),
                }
            )

        # Log summary and detection lines first (easier to read)
        if num_detections > 0:
            first_label = detected_objects[0]["class_label"]
            print(f"{req_num}. {dims} {object_id} {first_label}, {elapsed_ms:.1f}ms")
            print(f"✓ Detected {num_detections} object(s)")
            for obj in detected_objects:
                print(f"  - {obj['class']}: {obj['confidence']:.2f}")
        else:
            print(f"{req_num}. {dims} {object_id} no object, {elapsed_ms:.1f}ms")
            print("✗ No objects detected")

        save_original_image(image, filename)
        if num_detections > 0:
            saved_path = save_image_with_detections(image, results, filename)
            return (
                jsonify(
                    {
                        "success": True,
                        "detected": True,
                        "count": num_detections,
                        "objects": detected_objects,
                        "saved_path": saved_path,
                    }
                ),
                200,
            )
        else:
            return (
                jsonify(
                    {"success": False, "detected": False, "count": 0, "objects": []}
                ),
                200,
            )

    except Exception as e:
        print(f"Error during detection: {str(e)}")
        return jsonify({"success": False, "error": str(e)}), 500


@app.route("/")
def index():
    os.system(f'"{sys.executable}" display.py')
    return send_from_directory(".", "index.html")


@app.route("/detections/<path:filename>")
def serve_detections(filename):
    return send_from_directory("detections", filename)


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=4000, debug=True)
