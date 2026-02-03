from http.server import BaseHTTPRequestHandler, HTTPServer
import time
import cgi

class FakeImageServer(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/detect':
            # --- Parse the multipart form data to get the object_id ---
            form = cgi.FieldStorage(
                fp=self.rfile,
                headers=self.headers,
                environ={'REQUEST_METHOD': 'POST',
                         'CONTENT_TYPE': self.headers['Content-Type'],
                         })

            obstacle_id = "Unknown"
            if 'object_id' in form:
                obstacle_id = form['object_id'].value
            print(f"[Fake Img Server] Received image for obstacle ID: {obstacle_id}")

            # --- Simulate a long processing time ---
            print("[Fake Img Server] Simulating 5-second image recognition...")
            time.sleep(5)

            # --- Send the response ---
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            
            # Return a fake image ID (e.g., Obstacle 1 -> Image 11, Obstacle 2 -> Image 12)
            recognized_id = 10 + int(obstacle_id)
            response = f'{{"img_id": {recognized_id}}}'
            
            self.wfile.write(response.encode('utf-8'))
            print(f"[Fake Img Server] Sent response: {response}")
        else:
            self.send_response(404)
            self.end_headers()


def run_image_server():
    server_address = ('0.0.0.0', 5000)
    httpd = HTTPServer(server_address, FakeImageServer)
    print('Fake Image Recognition Server running on http://localhost:5000 ...')
    httpd.serve_forever()

if __name__ == '__main__':
    run_image_server()
