"""
Test script: send an image to the detection server and print the JSON response.

Usage (from Image folder, with server running on port 4000):
  python test_detect.py path/to/your/image.jpg
  python test_detect.py path/to/image.png 1

Optional second argument is object_id (default 1). Default URL: http://localhost:4000/detect
"""
import sys
import urllib.request
import urllib.error
import json

DEFAULT_URL = "http://localhost:4000/detect"


def main():
    if len(sys.argv) < 2:
        print("Usage: python test_detect.py <image_path> [object_id]")
        print("Example: python test_detect.py my_photo.jpg 1")
        sys.exit(1)

    image_path = sys.argv[1]
    object_id = sys.argv[2] if len(sys.argv) > 2 else "1"
    url = DEFAULT_URL

    try:
        with open(image_path, "rb") as f:
            image_data = f.read()
    except FileNotFoundError:
        print(f"Error: File not found: {image_path}")
        sys.exit(1)

    # Build multipart form: image + object_id
    boundary = b"----TestBoundary12345"
    body = (
        b"--" + boundary + b"\r\n"
        b'Content-Disposition: form-data; name="image"; filename="image.jpg"\r\n'
        b"Content-Type: image/jpeg\r\n\r\n"
    ) + image_data + (
        b"\r\n--" + boundary + b"\r\n"
        b'Content-Disposition: form-data; name="object_id"\r\n\r\n'
        + object_id.encode("utf-8") + b"\r\n"
        b"--" + boundary + b"--\r\n"
    )

    req = urllib.request.Request(
        url,
        data=body,
        method="POST",
        headers={"Content-Type": "multipart/form-data; boundary=" + boundary.decode("utf-8")},
    )

    try:
        with urllib.request.urlopen(req, timeout=60) as resp:
            response_body = resp.read().decode("utf-8")
            print("Status:", resp.status)
            try:
                data = json.loads(response_body)
                print("Response (JSON):")
                print(json.dumps(data, indent=2))
            except json.JSONDecodeError:
                print("Response (raw):", response_body[:500])
    except urllib.error.HTTPError as e:
        print("HTTP Error:", e.code, e.reason)
        print(e.read().decode("utf-8", errors="ignore")[:500])
        sys.exit(1)
    except urllib.error.URLError as e:
        print("URL Error:", e.reason)
        print("Make sure the server is running: python object_detection_server.py")
        sys.exit(1)


if __name__ == "__main__":
    main()
