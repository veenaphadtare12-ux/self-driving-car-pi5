"""
Camera source. On the Pi: picamera2 at 640x480. On the laptop: a video file,
an image, or a webcam index (for testing the pipeline without the car).

    cam = open_camera()              # picamera2
    cam = open_camera("clip.mp4")    # file / image / "0" for webcam
    frame = cam.read()               # BGR numpy array, or None when the source ends
    cam.close()
"""
import os

import cv2

import config


class PiCamera:
    def __init__(self):
        from picamera2 import Picamera2
        self.cam = Picamera2()
        cfg = self.cam.create_video_configuration(
            main={"size": (config.CAM_WIDTH, config.CAM_HEIGHT), "format": config.CAM_FORMAT},
            buffer_count=4,
        )
        self.cam.configure(cfg)
        self.cam.start()

    def read(self):
        # "RGB888" arrays are B,G,R order: already what OpenCV wants
        return self.cam.capture_array("main")

    def close(self):
        try:
            self.cam.stop()
            self.cam.close()
        except Exception:
            pass


class FileCamera:
    def __init__(self, source):
        self.image = None
        if isinstance(source, str) and os.path.splitext(source)[1].lower() in (".jpg", ".jpeg", ".png", ".bmp"):
            self.image = cv2.imread(source)
            if self.image is None:
                raise RuntimeError(f"cannot read image {source}")
            self.image = cv2.resize(self.image, (config.CAM_WIDTH, config.CAM_HEIGHT))
            self.cap = None
        else:
            src = int(source) if str(source).isdigit() else source
            self.cap = cv2.VideoCapture(src)
            if not self.cap.isOpened():
                raise RuntimeError(f"cannot open video source {source}")

    def read(self):
        if self.image is not None:
            return self.image.copy()
        ok, frame = self.cap.read()
        if not ok:
            return None
        if frame.shape[1] != config.CAM_WIDTH or frame.shape[0] != config.CAM_HEIGHT:
            frame = cv2.resize(frame, (config.CAM_WIDTH, config.CAM_HEIGHT))
        return frame

    def close(self):
        if self.cap is not None:
            self.cap.release()


def open_camera(source=None):
    return PiCamera() if source is None else FileCamera(source)
