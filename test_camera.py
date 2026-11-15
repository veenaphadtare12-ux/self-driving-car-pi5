"""
Stage 2: camera. Saves one photo.

    python test_camera.py                 # -> test_photo.jpg
    python test_camera.py -o track.jpg

Copy the photo to the laptop and tune the tape colour with tune_hsv.py.
"""
import argparse
import time

import cv2

from camera import open_camera


def main():
    p = argparse.ArgumentParser()
    p.add_argument("-o", "--out", default="test_photo.jpg")
    a = p.parse_args()

    cam = open_camera()
    try:
        time.sleep(1.5)  # let auto-exposure / white balance settle
        frame = cam.read()
        cv2.imwrite(a.out, frame)
        print(f"Saved {a.out}  shape={frame.shape}  dtype={frame.dtype}")
        print("If red and blue look swapped in the photo, CAM_FORMAT in config.py is wrong.")
    finally:
        cam.close()


if __name__ == "__main__":
    main()
