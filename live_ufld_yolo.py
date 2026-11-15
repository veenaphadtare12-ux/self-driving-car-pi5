"""Live browser view: UFLD lane points (green dots) + YOLO obstacle boxes. No motors.

Run with the car_project venv (it has onnxruntime + picamera2 + ultralytics):

    ~/car_project/venv/bin/python live_ufld_yolo.py            # open http://<pi-ip>:8000
    ~/car_project/venv/bin/python live_ufld_yolo.py --no-yolo

Camera is set up like the UFLD training captures (car_project/scripts/capture_app.py):
sensor 1296x972 (full field of view), main 1280x720 RGB888 (= BGR in memory), EV +1.
UFLD and YOLO each run in their own thread on the newest frame; the stream draws the
latest results on the latest frame, so the video stays smooth even though UFLD is ~5 FPS.
Ctrl+C to stop.
"""
import argparse
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
from picamera2 import Picamera2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.expanduser("~/car_project/scripts"))
import config  # noqa: E402
from lane_infer import LaneDetector as UFLD  # noqa: E402

UFLD_MODEL = os.path.expanduser("~/car_project/models/ufld_v2_same_mytrack.onnx")
W, H = 1280, 720
CENTER_ROW = 520          # row (of 720) where the lane centre / offset is measured

state = {"frame": None, "lanes": [{}, {}], "dets": [], "ufld_fps": 0.0, "yolo_fps": 0.0,
         "ufld_ms": 0.0, "yolo_ms": 0.0, "cam_fps": 0.0}
lock = threading.Lock()
stop = threading.Event()


def camera_loop(cam):
    n, t0 = 0, time.time()
    while not stop.is_set():
        f = cam.capture_array()[..., :3]
        with lock:
            state["frame"] = f
        n += 1
        if n % 30 == 0:
            now = time.time()
            state["cam_fps"] = 30 / (now - t0)
            t0 = now


def latest_frame():
    with lock:
        return state["frame"]


def ufld_loop(model):
    while not stop.is_set():
        f = latest_frame()
        if f is None:
            time.sleep(0.01)
            continue
        t = time.perf_counter()
        lanes = model.detect(f)
        dt = time.perf_counter() - t
        with lock:
            state["lanes"] = lanes
            state["ufld_ms"] = dt * 1000
            state["ufld_fps"] = 0.8 * state["ufld_fps"] + 0.2 / dt if state["ufld_fps"] else 1 / dt


def yolo_loop(det):
    while not stop.is_set():
        f = latest_frame()
        if f is None:
            time.sleep(0.01)
            continue
        t = time.perf_counter()
        dets = det.detect(f)
        dt = time.perf_counter() - t
        with lock:
            state["dets"] = dets
            state["yolo_ms"] = dt * 1000
            state["yolo_fps"] = 0.8 * state["yolo_fps"] + 0.2 / dt if state["yolo_fps"] else 1 / dt


def x_at(pts, y):
    """Lane x at image row y, interpolated from UFLD points {y: x}; None if not covered."""
    if len(pts) < 2:
        return None
    ys = np.array(sorted(pts))
    if y < ys[0] - 40 or y > ys[-1] + 40:
        return None
    xs = np.array([pts[k] for k in ys])
    return float(np.polyval(np.polyfit(ys, xs, 1), y))


def render(draw_dets):
    with lock:
        f = state["frame"]
        lanes, dets = state["lanes"], list(state["dets"])
        s = dict(state)
    if f is None:
        return None
    img = f.copy()
    for i, pts in enumerate(lanes[:2]):
        for y, x in pts.items():
            cv2.circle(img, (int(x), int(y)), 6, (0, 255, 0), -1)
            cv2.circle(img, (int(x), int(y)), 6, (0, 0, 0), 1)
        if pts:
            y0 = max(pts)
            cv2.putText(img, "L" if i == 0 else "R", (int(pts[y0]) - 10, y0 - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    # lane centre + offset
    cx_img = W // 2
    cv2.line(img, (cx_img, CENTER_ROW - 25), (cx_img, CENTER_ROW + 25), (255, 255, 255), 2)
    xl = x_at(lanes[0], CENTER_ROW) if lanes else None
    xr = x_at(lanes[1], CENTER_ROW) if len(lanes) > 1 else None
    off_txt = "lane: LOST"
    if xl is not None and xr is not None:
        c = (xl + xr) / 2
        cv2.circle(img, (int(c), CENTER_ROW), 10, (0, 255, 255), -1)
        cv2.arrowedLine(img, (cx_img, CENTER_ROW), (int(c), CENTER_ROW), (0, 255, 255), 3, tipLength=0.2)
        off_txt = f"offset {c - cx_img:+.0f}px (of 1280)"
    elif xl is not None or xr is not None:
        off_txt = "one line only (" + ("L" if xl is not None else "R") + ")"
    if draw_dets:
        from obstacle_yolo import draw_detections
        draw_detections(img, dets)
    hud = [f"UFLD {s['ufld_fps']:.1f} FPS ({s['ufld_ms']:.0f} ms)  pts L{len(lanes[0]) if lanes else 0}"
           f" R{len(lanes[1]) if len(lanes) > 1 else 0}",
           off_txt,
           (f"YOLO {s['yolo_fps']:.1f} FPS ({s['yolo_ms']:.0f} ms)  " +
            ", ".join(f"{d['cls']} {d['conf']:.2f}" for d in dets)) if draw_dets else "YOLO off",
           f"camera {s['cam_fps']:.0f} FPS"]
    cv2.rectangle(img, (0, 0), (W, 34 * len(hud) + 10), (0, 0, 0), -1)
    for i, t in enumerate(hud):
        cv2.putText(img, t, (12, 32 + 34 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2)
    return img


def make_handler(draw_dets):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path == "/":
                body = (b"<html><body style='background:#111;margin:0'>"
                        b"<img src='/stream' style='width:100%;max-width:1280px'></body></html>")
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
                while not stop.is_set():
                    img = render(draw_dets)
                    if img is not None:
                        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                        if ok:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                             + jpg.tobytes() + b"\r\n")
                    time.sleep(1 / 15)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--no-yolo", action="store_true")
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--ufld-threads", type=int, default=3)
    p.add_argument("--snapshot", metavar="OUT.jpg", help="save one rendered frame after ~8 s (for checks)")
    a = p.parse_args()

    print("Loading UFLD:", UFLD_MODEL)
    ufld = UFLD(UFLD_MODEL, threads=a.ufld_threads)
    det = None
    if not a.no_yolo:
        from obstacle_yolo import ObstacleDetector
        print("Loading YOLO:", config.YOLO_MODEL_PATH)
        det = ObstacleDetector()

    cam = Picamera2()
    cam.configure(cam.create_video_configuration(
        main={"size": (W, H), "format": "RGB888"},
        sensor={"output_size": (1296, 972), "bit_depth": 10},
        controls={"ExposureValue": 1.0, "AwbEnable": True}))
    cam.start()

    fans = None
    if det is not None:
        try:
            from fans import Fans
            fans = Fans()
            fans.on()
        except Exception as e:  # fans are nice-to-have here
            print("fans not used:", e)

    threads = [threading.Thread(target=camera_loop, args=(cam,), daemon=True),
               threading.Thread(target=ufld_loop, args=(ufld,), daemon=True)]
    if det is not None:
        threads.append(threading.Thread(target=yolo_loop, args=(det,), daemon=True))
    for t in threads:
        t.start()

    srv = ThreadingHTTPServer(("0.0.0.0", a.port), make_handler(det is not None))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"Live view: http://<pi-ip>:{a.port}   (Ctrl+C to stop)")
    try:
        t_start = time.time()
        while True:
            time.sleep(1)
            if a.snapshot and time.time() - t_start > 8:
                cv2.imwrite(a.snapshot, render(det is not None))
                print("snapshot saved:", a.snapshot)
                a.snapshot = None
            with lock:
                s = dict(state)
                nl = [len(x) for x in s["lanes"]]
            print(f"UFLD {s['ufld_fps']:.1f} FPS pts {nl} | YOLO {s['yolo_fps']:.1f} FPS "
                  f"{[d['cls'] for d in s['dets']]} | cam {s['cam_fps']:.0f}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        srv.shutdown()
        cam.stop()
        if fans is not None:
            fans.off()
        print("stopped")


if __name__ == "__main__":
    main()
