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


class ReplayCamera:
    """Replay for the threaded runtime / web page (--source): a folder of images (sorted) or a video file,
    delivered at `fps` like a camera (read() blocks until the next frame time), resized to CAM_WIDTH x
    CAM_HEIGHT, looping at the end when loop=True (else read() returns None at the end)."""

    IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")

    def __init__(self, source, fps=30.0, loop=True, clock=None):
        import time
        self._time = time
        self.clock = clock or time.monotonic
        self.source, self.fps, self.loop = source, float(fps), loop
        self.files, self.cap = None, None
        if os.path.isdir(source):
            self.files = sorted(os.path.join(source, f) for f in os.listdir(source)
                                if os.path.splitext(f)[1].lower() in self.IMG_EXT)
            if not self.files:
                raise RuntimeError(f"no images in {source}")
        else:
            self.cap = cv2.VideoCapture(source)
            if not self.cap.isOpened():
                raise RuntimeError(f"cannot open video source {source}")
        self.i = 0
        self.loops = 0
        self._next = None

    def _raw(self):
        if self.files is not None:
            if self.i >= len(self.files):
                if not self.loop:
                    return None
                self.i, self.loops = 0, self.loops + 1
            img = cv2.imread(self.files[self.i])
            self.i += 1
            return img
        ok, frame = self.cap.read()
        if not ok:
            if not self.loop:
                return None
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.loops += 1
            ok, frame = self.cap.read()
            if not ok:
                return None
        self.i += 1
        return frame

    def read(self):
        now = self.clock()
        if self._next is None:
            self._next = now
        elif self._next > now:
            self._time.sleep(self._next - now)
        else:
            self._next = max(self._next, now - 1.0 / self.fps)   # don't burst after a stall
        self._next += 1.0 / self.fps
        frame = self._raw()
        if frame is not None and (frame.shape[1] != config.CAM_WIDTH or frame.shape[0] != config.CAM_HEIGHT):
            frame = cv2.resize(frame, (config.CAM_WIDTH, config.CAM_HEIGHT))
        return frame

    def close(self):
        if self.cap is not None:
            self.cap.release()


def open_camera(source=None):
    return PiCamera() if source is None else FileCamera(source)
