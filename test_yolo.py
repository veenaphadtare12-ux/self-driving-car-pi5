"""
Stage 5: YOLO. Prints live detections and FPS. No motors.

    python test_yolo.py                        # picamera2
    python test_yolo.py --save yolo_frames     # also save annotated frames that have detections
    python test_yolo.py --source photo.jpg     # laptop
"""
import argparse
import os
import time

import cv2

import config
from camera import open_camera
from fans import Fans
from keys import KeyReader
from decision import in_corridor
from obstacle_yolo import ObstacleDetector, draw_detections


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", default=None)
    p.add_argument("--save", default=None, help="folder for annotated frames with detections")
    p.add_argument("--max", type=int, default=0)
    a = p.parse_args()

    if a.save:
        os.makedirs(a.save, exist_ok=True)
    fans = Fans()      # YOLO heats the Pi; no-op if the GPIO is unavailable
    fans.on()
    print(f"Loading {config.YOLO_MODEL_PATH} (imgsz={config.YOLO_IMGSZ}, conf={config.YOLO_CONF})...")
    cam = None
    try:
        det = ObstacleDetector()
        cam = open_camera(a.source)
        single_image = a.source is not None and a.source.lower().endswith((".jpg", ".jpeg", ".png", ".bmp"))
        i, fps = 0, 0.0
        with KeyReader() as keys:
            while True:
                frame = cam.read()
                if frame is None:
                    break
                t = time.monotonic()
                dets = det.detect(frame)
                dt = time.monotonic() - t
                fps = 1 / dt if fps == 0 else 0.9 * fps + 0.1 / dt
                txt = ", ".join(f"{d['cls']}({d['conf']:.2f}, h={d['box_height_ratio']:.2f}, "
                                f"x={d['cx_ratio']:.2f}{', CORRIDOR' if d['cls'] in config.OBSTACLE_CLASSES and in_corridor(d) else ''})"
                                for d in dets)
                print(f"[{i:5d}] {dt * 1000:6.1f} ms  {fps:4.1f} FPS  {txt or '-'}")
                if a.save and (dets or single_image):
                    cv2.imwrite(os.path.join(a.save, f"yolo_{i:05d}.jpg"), draw_detections(frame.copy(), dets))
                i += 1
                if single_image or (a.max and i >= a.max) or keys.get() == "q":
                    break
    except KeyboardInterrupt:
        print()
    finally:
        if cam is not None:
            cam.close()
        fans.cleanup()


if __name__ == "__main__":
    main()
