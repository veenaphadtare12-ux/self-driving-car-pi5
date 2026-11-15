"""
Stage 4: lane detection. Prints the offset live and saves overlay frames.
No motors. Push the car along the track by hand.

    python test_lane.py                       # picamera2, save every 10th overlay to lane_frames/
    python test_lane.py --every 5 --max 300
    python test_lane.py --source photo.jpg    # laptop: one image, saves one overlay
    python test_lane.py --source clip.mp4     # laptop: a recorded clip

Offset > 0: lane center is right of the image center. '-' = lane not found.
"""
import argparse
import os
import time

import cv2

from camera import open_camera
from keys import KeyReader
from lane_hough import LaneDetector


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", default=None)
    p.add_argument("--out", default="lane_frames")
    p.add_argument("--every", type=int, default=10, help="save every Nth overlay")
    p.add_argument("--max", type=int, default=0, help="stop after N frames")
    p.add_argument("--save-mask", action="store_true")
    a = p.parse_args()

    os.makedirs(a.out, exist_ok=True)
    cam = open_camera(a.source)
    lane = LaneDetector()
    single_image = a.source is not None and a.source.lower().endswith((".jpg", ".jpeg", ".png", ".bmp"))
    i, t0 = 0, time.monotonic()
    try:
        with KeyReader() as keys:
            while True:
                frame = cam.read()
                if frame is None:
                    break
                offset, overlay = lane.process(frame)
                fps = (i + 1) / max(time.monotonic() - t0, 1e-6)
                print(f"[{i:5d}] offset {'-' if offset is None else f'{offset:+6.1f}'} px "
                      f"({lane.last_method}) {fps:4.1f} FPS")
                if i % a.every == 0 or single_image:
                    cv2.imwrite(os.path.join(a.out, f"lane_{i:05d}.jpg"), overlay)
                    if a.save_mask:
                        cv2.imwrite(os.path.join(a.out, f"mask_{i:05d}.png"), lane.make_mask(frame))
                i += 1
                if single_image or (a.max and i >= a.max) or keys.get() == "q":
                    break
    except KeyboardInterrupt:
        print()
    finally:
        cam.close()
        print(f"Overlays in {a.out}/")


if __name__ == "__main__":
    main()
