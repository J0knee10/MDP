from http.server import BaseHTTPRequestHandler, HTTPServer
import time
import re
import json

class FakeImageServer(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/detect':
            content_length = int(self.headers['Content-Length'])
            body = self.rfile.read(content_length)
            body_str = body.decode('utf-8', errors='ignore')
            
            match = re.search(r'name="object_id"\r\n\r\n(\S+)', body_str)

            obstacle_id_str = "0"
            if match:
                obstacle_id_str = match.group(1)
            print(f"[Fake Img Server] Received image for obstacle ID: {obstacle_id_str}")

            # --- Simulate a long processing time ---
            print("[Fake Img Server] Simulating 5-second image recognition...")
            time.sleep(5)

            # --- Send the response ---
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            
            try:
                obstacle_id = int(obstacle_id_str)
                # Map typical classes for simulation
                if obstacle_id == 41:
                    class_label = "Bullseye"
                    img_id = 41
                else:
                    class_label = f"Number {obstacle_id}"
                    img_id = 10 + obstacle_id # Simple mapping for simulation
            except ValueError:
                class_label = "Unknown"
                img_id = -1

            # Match the format in Image/object_detection_server.py
            response_data = {
                "success": True,
                "detected": True,
                "count": 1,
                "objects": [
                    {
                        "class": f"{class_label} - {img_id}",
                        "class_label": class_label,
                        "class_id": str(img_id),
                        "img_id": img_id,
                        "confidence": 0.98,
                        "bbox": [50.0, 50.0, 150.0, 150.0]
                    }
                ],
                "saved_path": f"./detections/detection_{obstacle_id_str}.jpg"
            }
            
            # Create a compact JSON string without indentation.
            response_json = json.dumps(response_data)
            
            self.wfile.write(response_json.encode('utf-8'))
            print(f"[Fake Img Server] Sent response: {response_json}")
        else:
            self.send_response(404)
            self.end_headers()


def run_image_server():
    # Set to run on port 4000 as per user request.
    server_address = ('0.0.0.0', 4000)
    httpd = HTTPServer(server_address, FakeImageServer)
    print('Fake Image Recognition Server running on http://localhost:4000 ...')
    httpd.serve_forever()

if __name__ == '__main__':
    run_image_server()
