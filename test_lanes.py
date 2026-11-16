"""
Tests for the common lane interface (lanes/). No camera, no motors.

    python test_lanes.py -v

  * LaneResult sign convention: lane centre RIGHT of the image centre -> offset > 0 (both backends)
  * single-line fallback (centre = line +- half the learned lane width), width learning
  * empty input -> not found, offset None, lines "none"
  * smoothing, heading sign + bias, per-backend steering gains
  * UFLD pre/post-processing on synthetic data (colour order, invert, anchors); real model smoke test
"""
import glob
import math
import os
import unittest

import cv2
import numpy as np

import config
from decision import DecisionInputs, DecisionState, decide
from lanes import steer_cfg
from lanes.hough import HoughLane
from lanes.result import LaneEstimator

W_TABLE = {300: 1000, 500: 1000}      # constant 1000 px lane width (reference coordinates)


def est(**kw):
    args = dict(width_px_by_row=W_TABLE, lookahead_rows=(320, 420), heading_rows=(300, 480),
                image_center_x=640, smooth_frames=1, min_rows_in_band=3, min_confidence=0.45,
                single_line_confidence=0.8, width_learn_rate=0.5, width_scale_limits=(0.8, 1.2))
    args.update(kw)
    return LaneEstimator(**args)


def vertical(x, rows=range(300, 481, 20)):
    return {y: float(x) for y in rows}


class TestLaneResult(unittest.TestCase):
    def test_sign_lane_right_is_positive(self):
        r = est().update(vertical(240), vertical(1240))        # centre 740, image centre 640
        self.assertTrue(r.found)
        self.assertEqual(r.lines, "LR")
        self.assertAlmostEqual(r.center_x, 740)
        self.assertAlmostEqual(r.offset, 100 / 500)            # half width 500 -> +0.2

    def test_sign_lane_left_is_negative(self):
        r = est().update(vertical(40), vertical(1040))         # centre 540
        self.assertAlmostEqual(r.offset, -0.2)

    def test_centred_is_zero(self):
        self.assertAlmostEqual(est().update(vertical(140), vertical(1140)).offset, 0.0)

    def test_offset_clipped(self):
        self.assertEqual(est().update(vertical(1500), vertical(2500)).offset, 1.0)

    def test_single_left_line(self):
        r = est().update(vertical(300), {})
        self.assertTrue(r.found)
        self.assertEqual(r.lines, "L")
        self.assertAlmostEqual(r.center_x, 300 + 500)          # line + half width
        self.assertAlmostEqual(r.offset, 160 / 500)
        self.assertAlmostEqual(r.confidence, 0.8)

    def test_single_right_line(self):
        r = est().update({}, vertical(900))
        self.assertEqual(r.lines, "R")
        self.assertAlmostEqual(r.center_x, 400)
        self.assertAlmostEqual(r.offset, -240 / 500)

    def test_width_learned_from_two_line_frames(self):
        e = est(width_learn_rate=1.0)
        e.update(vertical(90), vertical(1190))                 # measured width 1100 = 1.1 x table
        self.assertAlmostEqual(e.width_scale, 1.1)
        r = e.update(vertical(300), {})                        # fallback now uses 1100 / 2
        self.assertAlmostEqual(r.center_x, 300 + 550)

    def test_width_scale_limited(self):
        e = est(width_learn_rate=1.0)
        e.update(vertical(0), vertical(2000))
        self.assertAlmostEqual(e.width_scale, 1.2)

    def test_empty_input(self):
        for left, right in (({}, {}), (None, None)):
            r = est().update(left, right)
            self.assertFalse(r.found)
            self.assertIsNone(r.offset)
            self.assertIsNone(r.center_x)
            self.assertEqual(r.lines, "none")
            self.assertEqual(r.confidence, 0.0)

    def test_lines_outside_band_not_found(self):
        r = est().update(vertical(140, rows=range(100, 200, 20)), vertical(1140, rows=range(100, 200, 20)))
        self.assertFalse(r.found)
        self.assertEqual(r.lines, "none")

    def test_too_few_rows_not_found(self):
        r = est().update({320: 140.0, 340: 140.0}, {320: 1140.0, 340: 1140.0})
        self.assertFalse(r.found)

    def test_low_confidence_not_found(self):
        self.assertFalse(est().update(vertical(140), vertical(1140), confidence=0.3).found)

    def test_smoothing_averages_valid_frames(self):
        e = est(smooth_frames=3)
        offs = [e.update(vertical(140 + d), vertical(1140 + d)).offset for d in (0, 150, 300)]
        self.assertAlmostEqual(offs[-1], (0 + 0.3 + 0.6) / 3)
        self.assertIsNone(e.update({}, {}).offset)             # a lost frame is not smoothed over

    def test_heading_sign_and_bias(self):
        # centre line moves right further ahead (smaller y -> larger x)
        left = {y: 140 + (480 - y) for y in range(300, 481, 20)}
        right = {y: 1140 + (480 - y) for y in range(300, 481, 20)}
        r = est().update(left, right)
        self.assertAlmostEqual(r.heading, math.atan(1.0))
        r2 = est(heading_bias=0.1).update(left, right)
        self.assertAlmostEqual(r2.heading, math.atan(1.0) - 0.1)


class TestSteeringGainsPerBackend(unittest.TestCase):
    def test_hough_uses_config_module(self):
        self.assertIsNone(steer_cfg("hough"))

    def test_ufld_gains(self):
        c = steer_cfg("ufld")
        self.assertEqual(c.STEER_KP, config.STEER_KP_UFLD)
        self.assertEqual(c.STEER_KD, config.STEER_KD_UFLD)
        self.assertEqual(c.STEER_MAX, config.STEER_MAX_UFLD)
        self.assertEqual(c.SAFETY_STOP_MM, config.SAFETY_STOP_MM)   # everything else unchanged
        out, _ = decide(DecisionInputs(t=0.0, offset_px=0.5), DecisionState(), c)
        self.assertAlmostEqual(out.steer, config.STEER_KP_UFLD * 0.5)
        self.assertGreater(out.left, out.right)                      # lane right -> steer right

    def test_ufld_heading_term(self):
        self.assertEqual(steer_cfg("ufld").STEER_KH, config.STEER_KH_UFLD)
        saved = config.STEER_KH_UFLD
        config.STEER_KH_UFLD = 10.0                                  # the page slider sets this live
        try:
            c = steer_cfg("ufld")
        finally:
            config.STEER_KH_UFLD = saved
        out, _ = decide(DecisionInputs(t=0.0, offset_px=0.0, heading=0.1), DecisionState(), c)
        self.assertAlmostEqual(out.steer, 10.0 * 0.1)
        self.assertGreater(out.left, out.right)                      # lane runs right ahead -> steer right
        out, _ = decide(DecisionInputs(t=0.0, offset_px=0.0, heading=-0.1), DecisionState(), c)
        self.assertLess(out.left, out.right)
        out, _ = decide(DecisionInputs(t=0.0, offset_px=0.2, heading=None), DecisionState(), c)
        self.assertAlmostEqual(out.steer, config.STEER_KP_UFLD * 0.2)  # unknown heading -> offset only

    def test_ufld_slowdown_in_turns(self):
        c = steer_cfg("ufld")
        out, _ = decide(DecisionInputs(t=0.0, offset_px=0.0), DecisionState(), c)
        self.assertEqual((out.left, out.right), (config.BASE_SPEED, config.BASE_SPEED))   # straight: full base
        out, _ = decide(DecisionInputs(t=0.0, offset_px=-10.0), DecisionState(), c)       # saturated left turn
        self.assertAlmostEqual(out.steer, -config.STEER_MAX_UFLD)
        base = config.BASE_SPEED * (1 - config.STEER_SLOWDOWN_UFLD)
        self.assertAlmostEqual(out.right, min(base + config.STEER_MAX_UFLD, config.MAX_SPEED))
        self.assertAlmostEqual(out.left, max(base - config.STEER_MAX_UFLD, config.MIN_WHEEL_SPEED))

    def test_hough_ignores_heading_and_slowdown(self):
        out, _ = decide(DecisionInputs(t=0.0, offset_px=30.0, heading=0.5), DecisionState())
        self.assertAlmostEqual(out.steer, config.STEER_KP * 30.0)
        self.assertAlmostEqual(out.left, config.BASE_SPEED + config.STEER_KP * 30.0)


def tape_image(shift):
    """White floor with two black tape lines converging upwards, shifted right by `shift` px."""
    img = np.full((config.CAM_HEIGHT, config.CAM_WIDTH, 3), 200, np.uint8)
    cx = 320 + shift
    cv2.line(img, (cx - 300, 340), (cx - 120, 120), (0, 0, 0), 14)
    cv2.line(img, (cx + 300, 340), (cx + 120, 120), (0, 0, 0), 14)
    return img


class TestHoughBackend(unittest.TestCase):
    def test_sign_both_ways(self):
        for shift, sign in ((60, 1), (-60, -1)):
            lane = HoughLane()
            off_px, _ = lane.process(tape_image(shift))
            r = lane.result
            self.assertIsNotNone(off_px)
            self.assertEqual(np.sign(off_px), sign, f"shift {shift}: offset {off_px}")
            self.assertEqual(np.sign(r.offset), sign)
            self.assertEqual(r.lines, "LR")
            self.assertEqual(r.source, "hough")
            self.assertAlmostEqual(r.offset, max(-1, min(1, off_px / (config.LANE_WIDTH_PX / 2))))

    def test_empty_image(self):
        lane = HoughLane()
        off, _ = lane.process(np.full((config.CAM_HEIGHT, config.CAM_WIDTH, 3), 200, np.uint8))
        self.assertIsNone(off)
        self.assertFalse(lane.result.found)
        self.assertEqual(lane.result.lines, "none")
        self.assertIsNone(lane.result.offset)


class TestUfldProcessing(unittest.TestCase):
    def model(self):
        from lanes.ufld import UfldModel, load_row_anchors
        m = UfldModel.__new__(UfldModel)          # no ONNX session needed for pre/post-processing
        m.resize, m.invert, m.anchors, m.conf_rows = "pil", True, load_row_anchors(), None
        m.col_w = 799 / 99
        return m

    def test_row_anchors(self):
        from lanes.ufld import load_row_anchors
        a = load_row_anchors()
        self.assertEqual(len(a), 56)
        self.assertEqual((a[0], a[-1]), (4, 284))

    def test_preprocess_invert_then_rgb(self):
        from lanes.ufld import MEAN, STD
        bgr = np.zeros((720, 1280, 3), np.uint8)
        bgr[..., 0] = 255                          # pure blue in BGR
        x = self.model().preprocess(bgr)
        self.assertEqual(x.shape, (1, 3, 288, 800))
        self.assertEqual(x.dtype, np.float32)
        # blue -> inverted (0,255,255) BGR = yellow -> RGB (255,255,0)
        expect = (np.array([1.0, 1.0, 0.0], np.float32) - MEAN) / STD
        np.testing.assert_allclose(x[0, :, 100, 100], expect, rtol=1e-5)

    def test_postprocess_points_and_no_lane(self):
        m = self.model()
        out = np.full((1, 101, 56, 2), -10.0, np.float32)
        out[0, 30, :, 0] = 10.0                    # lane 0: cell 30 at every anchor
        out[0, 100, :, 1] = 10.0                   # lane 1: "no lane" wins everywhere
        left, right = m.postprocess(out, 720, 1280)
        self.assertEqual(right, {})
        self.assertEqual(len(left), 56)
        x = 31 * m.col_w * 1280 / 800 - 1          # expectation = cell index + 1
        for v in left.values():
            self.assertAlmostEqual(v, x, places=3)
        self.assertEqual(sorted(left)[0], int(720 * 4 / 288) - 1)
        self.assertEqual(sorted(left)[-1], int(720 * 284 / 288) - 1)
        conf = m.confidences(out, [left, right], 720)
        self.assertGreater(conf[0], 0.99)
        self.assertEqual(conf[1], 0.0)

    @unittest.skipUnless(os.path.exists(config.UFLD_MODELS[config.UFLD_MODEL]), "UFLD model not copied")
    def test_real_model_on_a_track_frame(self):
        files = sorted(glob.glob(os.path.expanduser("~/car_project/dataset/lane/s04/*.jpg")))
        if not files:
            self.skipTest("no s04 frames")
        from lanes.ufld import UfldLane
        lane = UfldLane(threads=4)
        img = cv2.imread(files[len(files) // 2])
        off, overlay = lane.process(img, draw=True)
        r = lane.result
        self.assertEqual(overlay.shape, img.shape)
        self.assertTrue(r.found, f"lane not found: {r}")
        self.assertIn(r.lines, ("LR", "L", "R"))
        self.assertTrue(-1.0 <= off <= 1.0)
        self.assertEqual(r.source, "ufld:v2_same")


if __name__ == "__main__":
    unittest.main()
