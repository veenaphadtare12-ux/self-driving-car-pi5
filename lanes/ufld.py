"""UFLD v2 lane detection (ONNX Runtime), ported from ~/car_project/scripts/lane_infer.py and
car/perception.py. Preprocessing is EXACTLY the training one (notebooks/ufld_finetune_v2.ipynb):

  BGR frame (picamera2 "RGB888" = B,G,R in memory) -> invert (255 - pixel; black tape -> white lines)
  -> RGB -> PIL Image.resize((800, 288), BILINEAR) (= torchvision Resize((288, 800)) on a PIL image)
  -> /255 -> ImageNet mean/std -> NCHW float32 -> model (1x3x288x800 -> 1x101x56x2)
  -> softmax over the 100 grid cells -> expected cell -> x;  "no lane" class wins -> no point
  -> 56 row anchors (on the 288-px input, bottom-up output order) -> 2 lanes (left, right).

(Inverting before or after the BGR->RGB swap gives identical pixels; the order follows lane_infer.py.)

Line confidence = mean over the look-ahead band's anchors of P(line present) = 1 - P("no lane"),
as in car/perception.py; overall confidence = the better line.

Points are computed in reference coordinates (1280x720, the training frames). The camera's field of
view must match training (full sensor FOV, any size): the model input is always the whole frame
squeezed to 800x288, so a full-FOV 640x480 frame maps to the same geometry as 1280x720.
"""
import os
import re
import time

import cv2
import numpy as np

import config
from .result import LaneEstimator

IN_W, IN_H = 800, 288
GRID = 100
MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)
REF_W, REF_H = 1280, 720


def load_row_anchors(path=None):
    path = path or config.UFLD_ROW_ANCHORS
    with open(path) as f:
        txt = f.read()
    vals = sorted(int(round(float(x))) for x in re.findall(r"-?\d+\.?\d*", txt[txt.index("["):txt.index("]")]))
    if len(vals) != 56:
        raise ValueError(f"{path}: expected 56 row anchors, got {len(vals)}")
    return vals


class UfldModel:
    """Model + exact pre/post-processing. detect() returns ([left, right] {y: x}, [conf_l, conf_r])."""

    def __init__(self, model_path, threads=None, row_anchors=None, resize="pil", invert=True,
                 conf_rows=None):
        import onnxruntime as ort
        so = ort.SessionOptions()
        if threads:
            so.intra_op_num_threads = int(threads)
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.sess = ort.InferenceSession(model_path, so, providers=["CPUExecutionProvider"])
        self.input = self.sess.get_inputs()[0].name
        self.path = model_path
        self.threads = threads
        self.resize = resize
        self.invert = invert
        self.anchors = row_anchors or load_row_anchors()
        col = np.linspace(0, IN_W - 1, GRID)
        self.col_w = col[1] - col[0]
        self.conf_rows = conf_rows          # (y0, y1) in reference rows for the confidence, None = all
        self.times = {}

    # -- exactly lane_infer.py ---------------------------------------------------
    def preprocess(self, bgr):
        img = 255 - bgr if self.invert else bgr
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        if self.resize == "pil":
            from PIL import Image
            small = np.asarray(Image.fromarray(rgb).resize((IN_W, IN_H), Image.BILINEAR))
        else:
            small = cv2.resize(rgb, (IN_W, IN_H), interpolation={"linear": cv2.INTER_LINEAR,
                                                                 "area": cv2.INTER_AREA}[self.resize])
        x = small.astype(np.float32) / 255.0
        x = (x - MEAN) / STD
        return np.ascontiguousarray(x.transpose(2, 0, 1)[None])

    def infer(self, x):
        return self.sess.run(None, {self.input: x})[0]

    def postprocess(self, out, h, w):
        out = out[0][:, ::-1, :]                    # (101, 56, lanes), anchors bottom-up -> top-down
        logits = out[:-1]
        e = np.exp(logits - logits.max(axis=0, keepdims=True))
        prob = e / e.sum(axis=0, keepdims=True)
        loc = (prob * (np.arange(GRID) + 1).reshape(-1, 1, 1)).sum(axis=0)
        loc[np.argmax(out, axis=0) == GRID] = 0
        n = len(self.anchors)
        lanes = []
        for i in range(min(2, loc.shape[1])):
            pts = {}
            if np.sum(loc[:, i] != 0) > 2:
                for k in range(loc.shape[0]):
                    if loc[k, i] > 0:
                        pts[int(h * self.anchors[n - 1 - k] / IN_H) - 1] = float(loc[k, i] * self.col_w * w / IN_W - 1)
            lanes.append(pts)
        while len(lanes) < 2:
            lanes.append({})
        return lanes

    # -- confidence, as car/perception.py ------------------------------------------
    def confidences(self, out, lanes, h):
        o = out[0][:, ::-1, :]
        e = np.exp(o - o.max(axis=0, keepdims=True))
        p_none = e[-1] / e.sum(axis=0)              # (56, lanes)
        n = len(self.anchors)
        rows = np.array([int(h * self.anchors[n - 1 - k] / IN_H) - 1 for k in range(n)])
        band = np.ones(n, bool) if self.conf_rows is None else (rows >= self.conf_rows[0]) & (rows <= self.conf_rows[1])
        conf = []
        for i in range(2):
            if i >= p_none.shape[1] or not lanes[i] or not band.any():
                conf.append(0.0)
            else:
                conf.append(float(np.mean(1.0 - p_none[band, i])))
        return conf

    def detect(self, bgr, h=REF_H, w=REF_W):
        """Lane points in an h x w coordinate frame (default: reference 1280x720)."""
        t0 = time.perf_counter()
        x = self.preprocess(bgr)
        t1 = time.perf_counter()
        out = self.infer(x)
        t2 = time.perf_counter()
        lanes = self.postprocess(out, h, w)
        conf = self.confidences(out, lanes, h)
        t3 = time.perf_counter()
        self.times = {"pre": t1 - t0, "model": t2 - t1, "post": t3 - t2}
        return lanes, conf


def make_ufld_estimator():
    c = config
    return LaneEstimator(width_px_by_row=c.UFLD_WIDTH_PX_BY_ROW, lookahead_rows=c.UFLD_LOOKAHEAD_ROWS,
                         heading_rows=c.UFLD_HEADING_ROWS, image_center_x=c.UFLD_IMAGE_CENTER_X,
                         smooth_frames=c.LANE_SMOOTH_FRAMES, min_rows_in_band=c.UFLD_MIN_ROWS_IN_BAND,
                         min_confidence=c.UFLD_MIN_CONFIDENCE, single_line_confidence=c.UFLD_SINGLE_LINE_CONFIDENCE,
                         width_learn_rate=c.UFLD_WIDTH_LEARN_RATE, width_scale_limits=c.UFLD_WIDTH_SCALE_LIMITS,
                         heading_bias=c.LANE_HEADING_BIAS)


class UfldLane:
    """Lane backend. process(frame, draw) -> (normalised offset or None, overlay or None); .result = LaneResult."""
    backend = "ufld"

    def __init__(self, model=None, threads=None):
        self.model_name = model or config.UFLD_MODEL
        path = config.UFLD_MODELS[self.model_name]
        if not os.path.exists(path):
            raise FileNotFoundError(f"UFLD model missing: {path} (copy it from ~/car_project/models)")
        self.model = UfldModel(path, threads=threads or config.UFLD_THREADS, resize=config.UFLD_RESIZE,
                               conf_rows=config.UFLD_LOOKAHEAD_ROWS)
        self.estimator = make_ufld_estimator()
        self.source = f"ufld:{self.model_name}"
        self.result = None

    def reset(self):
        self.estimator.reset()

    def estimate(self, frame_bgr, t=None):
        h, w = frame_bgr.shape[:2]
        t = time.monotonic() if t is None else t
        lanes, conf = self.model.detect(frame_bgr)               # reference coordinates
        r = self.estimator.update(lanes[0], lanes[1], confidence=max(conf), t=t, source=self.source)
        # back to frame pixels for drawing / logging
        sx, sy = w / REF_W, h / REF_H
        r.left = {int(round(y * sy)): x * sx for y, x in r.left.items()}
        r.right = {int(round(y * sy)): x * sx for y, x in r.right.items()}
        r.center_x = None if r.center_x is None else r.center_x * sx
        r.band = (int(r.band[0] * sy), int(r.band[1] * sy))
        self.result = r
        return r

    def process(self, frame_bgr, draw=False):
        r = self.estimate(frame_bgr)
        return (r.offset if r.found else None), (draw_lane_result(frame_bgr, r) if draw else None)


def draw_lane_result(frame, r, dot_color=(0, 255, 0)):
    """Green dots for the lane points, look-ahead band, lane centre and offset arrow."""
    out = frame.copy()
    h, w = out.shape[:2]
    rad = max(2, w // 200)
    y0, y1 = r.band if r.band else (0, 0)
    if y1 > y0:
        cv2.rectangle(out, (0, y0), (w - 1, y1), (255, 255, 0), 1)
    for pts in (r.left, r.right):
        for y, x in pts.items():
            cv2.circle(out, (int(x), int(y)), rad, dot_color, -1)
    cx = w // 2
    ym = (y0 + y1) // 2 if y1 > y0 else int(h * 0.65)
    cv2.line(out, (cx, ym - 15), (cx, ym + 15), (255, 255, 255), 1)
    if r.found and r.center_x is not None:
        cv2.circle(out, (int(r.center_x), ym), rad + 3, (0, 255, 255), -1)
        cv2.arrowedLine(out, (cx, ym), (int(r.center_x), ym), (0, 255, 255), 2, tipLength=0.2)
    txt = (f"{r.source} {r.lines} off {r.offset:+.2f} conf {r.confidence:.2f}" if r.found
           else f"{r.source} lane: LOST ({r.lines}, conf {r.confidence:.2f})")
    cv2.putText(out, txt, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5 if w <= 800 else 0.8, (0, 255, 255), 2)
    return out
