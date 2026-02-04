from http.server import BaseHTTPRequestHandler, HTTPServer
import signal
import sys

class FakePathServer(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/path':
            print("[Fake Path Server] Received path request. Sending hardcoded route.")
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            # A long route with moves and snapshots to test concurrency
            response = """
            {
                "route": [
                    {"type":"FW", "value":10},
                    {"type":"TR", "value":90},
                    {"type":"SS", "value":1},
                    {"type":"FW", "value":15},
                    {"type":"TL", "value":90},
                    {"type":"SS", "value":2},
                    {"type":"FW", "value":20},
                    {"type":"SS", "value":3},
                    {"type":"FW", "value":5}
                ]
            }
            """
            self.wfile.write(response.encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

def run_path_server():
    server_address = ('0.0.0.0', 4000)
    httpd = HTTPServer(server_address, FakePathServer)
    print('Fake Pathfinding Server running on http://localhost:4000 ...')

    # Define a handler to gracefully shutdown
    def signal_handler(sig, frame):
        print("\n[Fake Path Server] KeyboardInterrupt received, shutting down server...")
        httpd.shutdown()  # stops serve_forever
        httpd.server_close()  # closes the socket
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Start serving
    httpd.serve_forever()

if __name__ == '__main__':
    run_path_server()
