"""
Obstacle detection with the fine-tuned YOLO26n NCNN export.

    det = ObstacleDetector()
    detections = det.detect(frame_bgr)
    # [{'cls': 'person', 'conf': 0.81, 'box': (x1, y1, x2, y2), 'box_height_ratio': 0.42,
    #   'cx_ratio': 0.51, 'bottom_ratio': 0.93}, ...]
    # cx_ratio = box center x / frame width, bottom_ratio = box bottom y / frame height

frame_bgr is a BGR numpy array (picamera2 "RGB888" is already BGR in memory).
"""
import cv2

import config

_COLORS = {
    "animal": (0, 165, 255), "car": (255, 0, 255), "light_green": (0, 255, 0),
    "light_red": (0, 0, 255), "person": (255, 128, 0), "stop_sign": (0, 0, 200),
}


class ObstacleDetector:
    def __init__(self, model_path=config.YOLO_MODEL_PATH, imgsz=config.YOLO_IMGSZ, conf=config.YOLO_CONF,
                 threads=None):
        """threads: NCNN CPU threads (None = NCNN default, all cores). NCNN reads it when the model is
        loaded (on the warm-up predict below), so it is set on the Net object ultralytics creates."""
        from ultralytics import YOLO
        self.model = YOLO(model_path, task="detect")
        self.imgsz = imgsz
        self.conf = conf
        self.names = self.model.names
        self.threads = threads
        # Warm-up: the first NCNN inference is slow because it loads the model
        import numpy as np
        restore = None
        if threads:
            import ncnn
            import torch
            torch.set_num_threads(int(threads))   # ultralytics pre/post-processing (process-wide)
            restore = ncnn.Net
            n = int(threads)

            class _Net(restore):
                def __init__(self):
                    super().__init__()
                    self.opt.num_threads = n
            ncnn.Net = _Net
        try:
            self.model.predict(np.zeros((config.CAM_HEIGHT, config.CAM_WIDTH, 3), np.uint8),
                               imgsz=self.imgsz, conf=self.conf, verbose=False)
        finally:
            if restore is not None:
                import ncnn
                ncnn.Net = restore

    def detect(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        r = self.model.predict(frame_bgr, imgsz=self.imgsz, conf=self.conf, verbose=False)[0]
        dets = []
        if r.boxes is None:
            return dets
        for xyxy, conf, cls in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist(), r.boxes.cls.tolist()):
            x1, y1, x2, y2 = (float(v) for v in xyxy)
            dets.append({
                "cls": self.names[int(cls)],
                "conf": float(conf),
                "box": (x1, y1, x2, y2),
                "box_height_ratio": (y2 - y1) / h,
                "cx_ratio": (x1 + x2) / 2.0 / w,
                "bottom_ratio": y2 / h,
            })
        return dets


def draw_detections(img, detections):
    for d in detections:
        x1, y1, x2, y2 = (int(v) for v in d["box"])
        col = _COLORS.get(d["cls"], (255, 255, 255))
        thick = 3 if d["cls"] in config.OBSTACLE_CLASSES or d["cls"] in config.TRAFFIC_CLASSES else 1
        cv2.rectangle(img, (x1, y1), (x2, y2), col, thick)
        label = f"{d['cls']} {d['conf']:.2f} h{d['box_height_ratio']:.2f}"
        cv2.putText(img, label, (x1, max(y1 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
    return img


def draw_regions(img):
    """Corridor (obstacles count only inside it) in yellow, signal region in magenta."""
    h, w = img.shape[:2]
    x0, x1 = (int(v * w) for v in config.CORRIDOR_X)
    yb = int(config.CORRIDOR_BOTTOM_MIN * h)
    cv2.rectangle(img, (x0, yb), (x1, h - 1), (0, 255, 255), 1)
    sx0, sx1 = (int(v * w) for v in config.SIGNAL_REGION_X)
    cv2.rectangle(img, (sx0, 60), (sx1 - 1, h - 1), (255, 0, 255), 1)
    return img


def format_detections(detections):
    """Compact string for the CSV log: cls:conf:height:center_x, e.g. 'person:0.81:0.42:0.51;...'."""
    return ";".join(f"{d['cls']}:{d['conf']:.2f}:{d['box_height_ratio']:.2f}:{d.get('cx_ratio', 0):.2f}"
                    for d in detections)
