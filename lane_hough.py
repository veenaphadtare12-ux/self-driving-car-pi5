"""
Tape lane detection: HSV mask -> ROI -> Canny/Hough, with a mask-centroid fallback.

    lane = LaneDetector()
    offset_px, overlay = lane.process(frame_bgr)

offset_px > 0 means the lane center is RIGHT of the image center (steer right).
offset_px is None when no lane is found. overlay is a BGR debug image, or None
when draw=False (skipping the drawing saves CPU when nothing is displayed).
"""
import cv2
import numpy as np

import config


class LaneDetector:
    def __init__(self, cfg=config):
        self.cfg = cfg
        self.last_center = None   # used to split left/right lines on curves
        self.last_method = "none"
        self.last_fits = (None, None)
        k = cfg.MORPH_KERNEL
        self._kernel = np.ones((k, k), np.uint8)

    # ------------------------------------------------------------------
    def make_mask(self, frame_bgr):
        c = self.cfg
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(c.HSV_LOWER, np.uint8), np.array(c.HSV_UPPER, np.uint8))
        roi = np.zeros_like(mask)
        roi[c.ROI_TOP:c.ROI_BOTTOM, :] = 255
        mask = cv2.bitwise_and(mask, roi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)
        return mask

    def process(self, frame_bgr, draw=True):
        c = self.cfg
        h, w = frame_bgr.shape[:2]
        img_cx = w / 2.0
        divider = self.last_center if self.last_center is not None else img_cx

        mask = self.make_mask(frame_bgr)
        center = None
        left_fit = right_fit = None
        segments = []

        if cv2.countNonZero(mask) >= c.MIN_MASK_PIXELS:
            if c.LANE_MODE == "center_line":
                center = self._centroid_center_line(mask)
                self.last_method = "centroid" if center is not None else "none"
            else:
                center, left_fit, right_fit, segments = self._hough_two_lines(mask, divider)
                if center is not None:
                    self.last_method = "hough"
                else:
                    center = self._centroid_two_lines(mask, divider)
                    self.last_method = "centroid" if center is not None else "none"
        else:
            self.last_method = "none"

        self.last_center = center  # None resets the divider to the image center next frame
        self.last_fits = (left_fit, right_fit)   # read by lanes/hough.py for LaneResult
        offset = None if center is None else float(center - img_cx)

        overlay = None
        if draw:
            overlay = self._draw(frame_bgr, mask, segments, left_fit, right_fit, center, offset)
        return offset, overlay

    # ------------------------------------------------------------------
    def _hough_two_lines(self, mask, divider):
        c = self.cfg
        edges = cv2.Canny(mask, c.CANNY_LOW, c.CANNY_HIGH)
        lines = cv2.HoughLinesP(edges, c.HOUGH_RHO, np.pi / 180, c.HOUGH_THRESHOLD,
                                minLineLength=c.HOUGH_MIN_LINE_LEN, maxLineGap=c.HOUGH_MAX_LINE_GAP)
        if lines is None:
            return None, None, None, []

        y_look = c.LOOKAHEAD_ROW
        left_pts, right_pts, kept = [], [], []
        # OpenCV 4.x returns (N,1,4), 5.x returns (N,4)
        for x1, y1, x2, y2 in lines.reshape(-1, 4).astype(float):
            if y1 == y2:
                continue
            slope = (y2 - y1) / (x2 - x1 + 1e-6)
            if abs(slope) < c.HOUGH_MIN_ABS_SLOPE:
                continue
            max_slope = getattr(c, "HOUGH_MAX_ABS_SLOPE", None)
            if max_slope is not None and abs(slope) > max_slope:
                continue  # near-vertical clutter (poles, obstacle edges)
            # x at the lookahead row for this segment (x = a*y + b)
            a = (x2 - x1) / (y2 - y1)
            x_look = x1 + a * (y_look - y1)
            side = left_pts if x_look < divider else right_pts
            side.extend([(x1, y1), (x2, y2)])
            kept.append((x1, y1, x2, y2, side is left_pts))

        left_fit = self._fit(left_pts)
        right_fit = self._fit(right_pts)
        xl = self._eval(left_fit, y_look)
        xr = self._eval(right_fit, y_look)

        if xl is not None and xr is not None and xr > xl:
            center = (xl + xr) / 2.0
        elif xl is not None and xr is None:
            center = xl + c.LANE_WIDTH_PX / 2.0
        elif xr is not None and xl is None:
            center = xr - c.LANE_WIDTH_PX / 2.0
        else:
            center = None
        return center, left_fit, right_fit, kept

    @staticmethod
    def _fit(pts):
        if len(pts) < 2:
            return None
        pts = np.array(pts, dtype=np.float32)
        ys, xs = pts[:, 1], pts[:, 0]
        if np.ptp(ys) < 5:
            return None
        return np.polyfit(ys, xs, 1)  # x = a*y + b

    @staticmethod
    def _eval(fit, y):
        return None if fit is None else float(fit[0] * y + fit[1])

    def _centroid_two_lines(self, mask, divider):
        c = self.cfg
        band = mask[c.ROI_TOP:c.ROI_BOTTOM, :]
        ys, xs = np.nonzero(band)
        if len(xs) < c.MIN_MASK_PIXELS:
            return None
        lx, rx = xs[xs < divider], xs[xs >= divider]
        min_px = c.MIN_MASK_PIXELS // 3
        has_l, has_r = len(lx) >= min_px, len(rx) >= min_px
        if has_l and has_r:
            return (lx.mean() + rx.mean()) / 2.0
        if has_l:
            return lx.mean() + c.LANE_WIDTH_PX / 2.0
        if has_r:
            return rx.mean() - c.LANE_WIDTH_PX / 2.0
        return None

    def _centroid_center_line(self, mask):
        c = self.cfg
        # Prefer a band around the lookahead row; fall back to the whole ROI
        for top, bot in ((c.LOOKAHEAD_ROW - 20, c.LOOKAHEAD_ROW + 20), (c.ROI_TOP, c.ROI_BOTTOM)):
            m = cv2.moments(mask[max(top, 0):bot, :], binaryImage=True)
            if m["m00"] >= c.MIN_MASK_PIXELS // 3:
                return m["m10"] / m["m00"]
        return None

    # ------------------------------------------------------------------
    def _draw(self, frame, mask, segments, left_fit, right_fit, center, offset):
        c = self.cfg
        h, w = frame.shape[:2]
        out = frame.copy()
        tint = np.zeros_like(out)
        tint[mask > 0] = (0, 255, 0)
        out = cv2.addWeighted(out, 1.0, tint, 0.5, 0)

        cv2.rectangle(out, (0, c.ROI_TOP), (w - 1, c.ROI_BOTTOM - 1), (255, 255, 0), 1)
        cv2.line(out, (0, c.LOOKAHEAD_ROW), (w, c.LOOKAHEAD_ROW), (128, 128, 128), 1)

        for x1, y1, x2, y2, is_left in segments:
            cv2.line(out, (int(x1), int(y1)), (int(x2), int(y2)),
                     (255, 0, 0) if is_left else (0, 0, 255), 2)
        for fit, col in ((left_fit, (255, 128, 0)), (right_fit, (0, 128, 255))):
            if fit is not None:
                x_top, x_bot = self._eval(fit, c.ROI_TOP), self._eval(fit, c.ROI_BOTTOM)
                cv2.line(out, (int(x_top), c.ROI_TOP), (int(x_bot), c.ROI_BOTTOM), col, 3)

        cx = w // 2
        cv2.line(out, (cx, c.ROI_TOP), (cx, c.ROI_BOTTOM), (255, 255, 255), 1)
        if center is not None:
            cv2.circle(out, (int(center), c.LOOKAHEAD_ROW), 8, (0, 255, 255), -1)
            cv2.arrowedLine(out, (cx, c.LOOKAHEAD_ROW), (int(center), c.LOOKAHEAD_ROW),
                            (0, 255, 255), 2, tipLength=0.2)
        txt = "lane: LOST" if offset is None else f"offset {offset:+.0f}px ({self.last_method})"
        cv2.putText(out, txt, (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return out
