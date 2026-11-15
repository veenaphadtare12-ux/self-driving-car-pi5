"""Hough backend: the existing lane_hough.LaneDetector, unchanged, plus a LaneResult for display/logging.

The offset given to decision.py stays EXACTLY what LaneDetector returns (pixels, with its own
single-line fallback via LANE_WIDTH_PX), so the calibrated Hough behaviour and STEER_KP/STEER_KD
are untouched. LaneResult.offset is that pixel offset normalised by half the lane width
(LANE_WIDTH_PX / 2 at LOOKAHEAD_ROW), clipped to -1..+1; heading comes from the mean slope of the
fitted lines minus LANE_HEADING_BIAS.
"""
import math
import time

import config
from lane_hough import LaneDetector

from .result import LaneResult, lines_label


class HoughLane:
    backend = "hough"
    source = "hough"

    def __init__(self, cfg=config):
        self.cfg = cfg
        self.det = LaneDetector(cfg)
        self.result = None

    def reset(self):
        self.det.last_center = None

    def process(self, frame_bgr, draw=False):
        offset_px, overlay = self.det.process(frame_bgr, draw=draw)
        self.result = self.to_result(offset_px, frame_bgr.shape[1])
        return offset_px, overlay

    def to_result(self, offset_px, width, t=None):
        c = self.cfg
        lf, rf = getattr(self.det, "last_fits", (None, None))
        rows = range(c.ROI_TOP, c.ROI_BOTTOM + 1, 10)
        left = {} if lf is None else {y: float(lf[0] * y + lf[1]) for y in rows}
        right = {} if rf is None else {y: float(rf[0] * y + rf[1]) for y in rows}
        method = self.det.last_method
        if lf is None and rf is None and offset_px is not None:
            lines = "LR" if method == "centroid" else "none"   # centroid fallback: no fitted lines
        else:
            lines = lines_label(lf is not None, rf is not None)
        found = offset_px is not None
        half = c.LANE_WIDTH_PX / 2.0
        norm = None if not found else max(-1.0, min(1.0, offset_px / half))
        heading = None
        slopes = [f[0] for f in (lf, rf) if f is not None]
        if found and slopes:
            heading = math.atan(-sum(slopes) / len(slopes)) - c.LANE_HEADING_BIAS
        conf = 0.0 if not found else (1.0 if lines == "LR" else 0.8)
        return LaneResult(left=left, right=right, found=found,
                          center_x=None if not found else width / 2.0 + offset_px,
                          offset=norm, raw_offset=norm, heading=heading, confidence=conf, lines=lines,
                          t=time.monotonic() if t is None else t, source="hough",
                          band=(c.ROI_TOP, c.ROI_BOTTOM))
