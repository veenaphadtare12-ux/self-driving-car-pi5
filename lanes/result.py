"""Common lane result + the lane estimator (ported from ~/car_project/car/lane_estimator.py).

Sign convention (same as test_lane.py and lane_hough.py):
  offset  > 0  the lane centre is RIGHT of the image centre -> steer right. Normalised -1..+1,
               1 = half a lane width.
  heading > 0  the lane centre line runs to the right further ahead (radians, image-space angle),
               minus LANE_HEADING_BIAS (camera / mount yaw).

The estimator works in REFERENCE coordinates (default 1280x720, the UFLD training frames), so the
band rows and width table don't depend on the camera resolution; callers scale points first.
  * look-ahead band: offset = mean lane centre over rows lookahead_rows
  * single line: centre = visible line +- half the lane width at that row (width table, corrected online
    by an EMA of the measured width in two-line frames, limited to width_scale_limits)
  * smoothing: offset/heading averaged over the last smooth_frames valid frames
"""
import collections
import math
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

import numpy as np


@dataclass
class LaneResult:
    left: Dict[int, float] = field(default_factory=dict)     # {row y: x} in frame pixels
    right: Dict[int, float] = field(default_factory=dict)
    found: bool = False
    center_x: Optional[float] = None    # lane centre x in frame pixels (look-ahead band)
    offset: Optional[float] = None      # -1..+1, > 0 = lane centre right of image centre (smoothed)
    raw_offset: Optional[float] = None  # this frame only
    heading: Optional[float] = None     # radians, bias removed (smoothed)
    confidence: float = 0.0
    lines: str = "none"                 # "LR", "L", "R" or "none"
    t: float = 0.0                      # time.monotonic() of the frame
    source: str = ""                    # "hough", "ufld:v2_same", ...
    band: tuple = ()                    # look-ahead rows in frame pixels (y0, y1)
    width_scale: float = 1.0


def lines_label(has_l, has_r):
    return ("L" if has_l else "") + ("R" if has_r else "") or "none"


class LaneEstimator:
    def __init__(self, *, width_px_by_row, lookahead_rows, heading_rows, image_center_x,
                 smooth_frames=3, min_rows_in_band=3, min_confidence=0.45, single_line_confidence=0.8,
                 width_learn_rate=0.05, width_scale_limits=(0.8, 1.2), heading_bias=0.0):
        rows = sorted(int(k) for k in width_px_by_row)
        if len(rows) < 2:
            raise ValueError("width_px_by_row needs at least 2 rows")
        self._w_rows = np.array(rows, float)
        self._w_vals = np.array([width_px_by_row[r] for r in rows], float)
        self.lookahead_rows = tuple(lookahead_rows)
        self.heading_rows = tuple(heading_rows)
        self.image_center_x = float(image_center_x)
        self.min_rows_in_band = int(min_rows_in_band)
        self.min_confidence = float(min_confidence)
        self.single_line_confidence = float(single_line_confidence)
        self.width_learn_rate = float(width_learn_rate)
        self.width_scale_limits = tuple(width_scale_limits)
        self.heading_bias = float(heading_bias)
        self.width_scale = 1.0
        self.history = collections.deque(maxlen=max(1, int(smooth_frames)))

    def reset(self):
        self.history.clear()

    def width(self, y):
        """Lane width in px at row y (linear inter/extrapolation of the table) x learned scale."""
        r, v = self._w_rows, self._w_vals
        if y <= r[0]:
            w = v[0] + (y - r[0]) * (v[1] - v[0]) / (r[1] - r[0])
        elif y >= r[-1]:
            w = v[-1] + (y - r[-1]) * (v[-1] - v[-2]) / (r[-1] - r[-2])
        else:
            w = float(np.interp(y, r, v))
        return w * self.width_scale

    def _centres(self, left, right, y0, y1, learn):
        pts, ratios = {}, []
        for y in sorted({y for y in list(left) + list(right) if y0 <= y <= y1}):
            xl, xr = left.get(y), right.get(y)
            if xl is not None and xr is not None and xr > xl:
                pts[y] = (xl + xr) / 2
                ratios.append((xr - xl) / (self.width(y) / self.width_scale))
            elif xl is not None and xr is None:
                pts[y] = xl + self.width(y) / 2
            elif xr is not None and xl is None:
                pts[y] = xr - self.width(y) / 2
        if learn and ratios:
            lo, hi = self.width_scale_limits
            k = self.width_learn_rate
            self.width_scale = float(np.clip((1 - k) * self.width_scale + k * np.median(ratios), lo, hi))
        return pts

    def update(self, left, right, confidence=1.0, t=None, source=""):
        """left/right: {row y: x} in reference coordinates. confidence: backend's 0..1 line confidence.
        Returns a LaneResult in reference coordinates (the backend rescales points for drawing)."""
        left, right = left or {}, right or {}
        y0, y1 = self.lookahead_rows
        band = self._centres(left, right, y0, y1, learn=True)
        h0, h1 = self.heading_rows
        hpts = self._centres(left, right, h0, h1, learn=False)

        has_l = any(y0 <= y <= y1 for y in left)
        has_r = any(y0 <= y <= y1 for y in right)
        conf = float(confidence) * (1.0 if (has_l and has_r) else self.single_line_confidence)
        if not (has_l or has_r):
            conf = 0.0
        found = len(band) >= self.min_rows_in_band and conf >= self.min_confidence

        raw_offset = heading = lane_x = None
        if band:
            lane_x = float(np.mean(list(band.values())))
            ymid = (y0 + y1) / 2
            raw_offset = float(np.clip((lane_x - self.image_center_x) / (self.width(ymid) / 2), -1.0, 1.0))
        if len(hpts) >= 3 and (max(hpts) - min(hpts)) >= 60:
            ys = np.array(sorted(hpts), float)
            xs = np.array([hpts[y] for y in sorted(hpts)])
            a = np.polyfit(ys, xs, 1)[0]            # dx/dy; far = smaller y
            heading = float(math.atan(-a)) - self.heading_bias
        if found:
            self.history.append((raw_offset, heading if heading is not None else 0.0))
            offset = float(np.mean([o for o, _ in self.history]))
            head = float(np.mean([hh for _, hh in self.history]))
        else:
            offset, head = None, heading
        return LaneResult(left=dict(left), right=dict(right), found=found,
                          center_x=lane_x if found else None, offset=offset, raw_offset=raw_offset,
                          heading=head, confidence=conf, lines=lines_label(has_l, has_r),
                          t=time.monotonic() if t is None else t, source=source,
                          band=(y0, y1), width_scale=self.width_scale)
