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
    "data": {
        "commands": [
            "FW10",
            "TR90",
            "SP1",
            "FW15",
            "TL90",
            "SP2",
            "FW20",
            "SP3",
            "FW5"
        ],
        "path": [
            {"x": 0, "y": 0, "d": 0, "s": -1},
            {"x": 0, "y": 1, "d": 0, "s": -1},
            {"x": 1, "y": 1, "d": 2, "s": -1},
            {"x": 1, "y": 2, "d": 2, "s": 1},
            {"x": 2, "y": 2, "d": 0, "s": -1},
            {"x": 2, "y": 3, "d": 0, "s": 2},
            {"x": 3, "y": 3, "d": 2, "s": -1},
            {"x": 3, "y": 4, "d": 2, "s": 3},
            {"x": 4, "y": 4, "d": 0, "s": -1}
        ],
        "distance": 100.0,
        "snap_positions": [
            {"x": 1, "y": 2, "d": 2},
            {"x": 2, "y": 3, "d": 0},
            {"x": 3, "y": 4, "d": 2}
        ]
    }
}
            """
            self.wfile.write(response.encode('utf-8'))
        else:
            self.send_response(404)
            self.end_headers()

def run_path_server():
    server_address = ('0.0.0.0', 5000) # Changed to port 5000
    httpd = HTTPServer(server_address, FakePathServer)
    print('Fake Pathfinding Server running on http://localhost:5000 ...') # Changed port in print
