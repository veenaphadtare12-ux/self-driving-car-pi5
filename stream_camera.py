"""Live camera preview in a browser (MJPEG). Ctrl+C to stop.

    python stream_camera.py            # then open http://<pi-ip>:8000
    python stream_camera.py --flip     # preview rotated 180 deg (camera mounted upside down)

A horizontal line marks the image centre; the bottom half is what the lane code mostly uses.
"""
import argparse
import io
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
from picamera2 import Picamera2

import config

latest = None
lock = threading.Lock()


def capture_loop(flip):
    global latest
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (config.CAM_WIDTH, config.CAM_HEIGHT), "format": config.CAM_FORMAT}))
    cam.start()
    while True:
        frame = cam.capture_array()  # BGR for OpenCV
        if flip:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        h, w = frame.shape[:2]
        cv2.line(frame, (0, h // 2), (w, h // 2), (0, 255, 255), 1)
        cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 255, 255), 1)
        cv2.putText(frame, "TOP", (w // 2 + 5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with lock:
                latest = jpg.tobytes()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            body = b"<html><body style='margin:0;background:#111'>" \
                   b"<img src='/stream' style='width:100%;max-width:960px'></body></html>"
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with lock:
                    jpg = latest
                if jpg:
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                time.sleep(1 / 20)
        except OSError:  # viewer left or network dropped; keep the server running
            pass

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--flip", action="store_true", help="rotate preview 180 deg")
    a = p.parse_args()
    threading.Thread(target=capture_loop, args=(a.flip,), daemon=True).start()
    print(f"Streaming on http://0.0.0.0:{a.port}  (Ctrl+C to stop)")
    try:
        ThreadingHTTPServer(("0.0.0.0", a.port), Handler).serve_forever()
    except KeyboardInterrupt:
        pass
