"""
Unit + scenario tests for decision.py with fake inputs. No hardware. Run on the laptop or the Pi:

    python test_decision.py -v

Times are seconds. Each step with detections and no det_t is a new YOLO check at time t;
each sensor value with no *_t is a new reading at time t.
"""
import math
import types
import unittest

import config
from decision import (CAUTION, DRIVE, ESTOP, STOP_LANE_LOST, STOP_OBSTACLE, STOP_RED, STOP_SIGN,
                      DecisionInputs, DecisionState, decide)


def cfg_with(**overrides):
    c = types.SimpleNamespace(**{k: getattr(config, k) for k in dir(config) if k.isupper()})
    for k, v in overrides.items():
        setattr(c, k, v)
    return c


def det(cls="person", h=0.2, cx=0.5, bottom=0.9, conf=0.8):
    """Fake YOLO detection. Defaults: centered, bottom of frame (inside the corridor)."""
    W, H = config.CAM_WIDTH, config.CAM_HEIGHT
    x1, x2 = (cx - 0.05) * W, (cx + 0.05) * W
    y2 = bottom * H
    return {"cls": cls, "conf": conf, "box": (x1, y2 - h * H, x2, y2),
            "box_height_ratio": h, "cx_ratio": cx, "bottom_ratio": bottom}


def red(h=0.2, cx=0.8, conf=0.9):
    return det("light_red", h=h, cx=cx, bottom=0.4, conf=conf)


def green(h=0.2, cx=0.8, conf=0.9):
    return det("light_green", h=h, cx=cx, bottom=0.4, conf=conf)


def sign(h=None, conf=0.9):
    return det("stop_sign", h=config.STOP_SIGN_HEIGHT_RATIO + 0.05 if h is None else h, cx=0.8, conf=conf)


STOP = config.STOP_DISTANCE_MM
NEAR = (config.SAFETY_STOP_MM + STOP) / 2                       # 300: inside stop distance, outside safety net
MID = (STOP + config.CAUTION_DISTANCE_MM) / 2                    # 600: caution band
FAR = config.CAUTION_DISTANCE_MM + 500                           # 1300
RESUME = STOP + config.RESUME_MARGIN_MM                          # 500
CLEAR = config.OBSTACLE_CLEAR_S
SAFE = config.SAFETY_STOP_MM
N_SIG = config.SIGNAL_CONFIRM_CHECKS
N_SIGN = config.STOP_SIGN_CONFIRM_CHECKS
DT = 0.1                                                         # frame/check spacing in scenarios


def I(t, offset=0.0, **kw):
    return DecisionInputs(t=t, offset_px=offset, **kw)


def run(steps, st=None, cfg=config):
    """steps: list of DecisionInputs. Returns (list of outputs, final state)."""
    st = st or DecisionState()
    outs = []
    for inp in steps:
        out, st = decide(inp, st, cfg)
        outs.append(out)
    return outs, st


def states(outs):
    return [o.state for o in outs]


# ===========================================================================
# Steering / lane (unchanged rules)
# ===========================================================================
class TestSteering(unittest.TestCase):
    def test_centered_drives_straight(self):
        out, _ = decide(I(0, tof_mm=FAR), DecisionState())
        self.assertEqual(out.state, DRIVE)
        self.assertEqual((out.left, out.right), (config.BASE_SPEED, config.BASE_SPEED))

    def test_lane_right_turns_right(self):
        out, _ = decide(I(0, 30.0), DecisionState())
        self.assertGreater(out.left, out.right)
        self.assertAlmostEqual(out.steer, config.STEER_KP * 30.0)

    def test_lane_left_turns_left(self):
        out, _ = decide(I(0, -30.0), DecisionState())
        self.assertLess(out.left, out.right)

    def test_speeds_clamped(self):
        out, _ = decide(I(0, 10_000.0), DecisionState())
        self.assertLessEqual(out.left, config.MAX_SPEED)
        self.assertGreaterEqual(out.right, config.MIN_WHEEL_SPEED)

    def test_derivative_term(self):
        outs, _ = run([I(0.0, 10.0), I(0.1, 20.0)])
        expected = config.STEER_KP * 20.0 + config.STEER_KD * 100.0
        self.assertAlmostEqual(outs[1].steer, min(expected, config.STEER_MAX), places=6)

    def test_no_derivative_spike_after_reacquire(self):
        outs, _ = run([I(0.0, -100.0), I(0.1, None), I(0.2, 50.0)])
        self.assertAlmostEqual(outs[2].steer, config.STEER_KP * 50.0)


class TestLaneLost(unittest.TestCase):
    def test_lane_lost_after_n_frames_and_recovers(self):
        n = config.LANE_LOST_FRAMES
        steps = [I(0.0, 20.0)] + [I(0.05 * (i + 1), None) for i in range(n)] + [I(0.05 * (n + 2), 0.0)]
        outs, _ = run(steps)
        for o in outs[1:n]:
            self.assertEqual(o.state, DRIVE)
            self.assertAlmostEqual(o.steer, outs[0].steer)
        self.assertEqual(outs[n].state, STOP_LANE_LOST)
        self.assertEqual(outs[-1].state, DRIVE)

    def test_lane_lost_beats_caution(self):
        st = DecisionState(lost_frames=config.LANE_LOST_FRAMES)
        out, _ = decide(I(0, None, detections=[det(h=0.35)], tof_mm=MID), st)
        self.assertEqual(out.state, STOP_LANE_LOST)


# ===========================================================================
# Obstacles: corridor + ToF / vision fallback
# ===========================================================================
class TestObstacleScenarios(unittest.TestCase):
    def test_person_in_corridor_close_stops(self):
        out, _ = decide(I(0, detections=[det("person")], tof_mm=NEAR), DecisionState())
        self.assertEqual(out.state, STOP_OBSTACLE)
        self.assertEqual((out.left, out.right), (0.0, 0.0))

    def test_all_obstacle_classes_stop(self):
        for cls in config.OBSTACLE_CLASSES:
            out, _ = decide(I(0, detections=[det(cls)], tof_mm=NEAR), DecisionState())
            self.assertEqual(out.state, STOP_OBSTACLE, cls)

    def test_person_beside_track_drives(self):
        for cx in (0.1, 0.2, 0.8, 0.95):
            out, _ = decide(I(0, detections=[det("person", h=0.6, cx=cx)], tof_mm=NEAR), DecisionState())
            self.assertEqual(out.state, DRIVE, f"cx={cx}")

    def test_person_high_in_frame_drives(self):
        # box bottom in the upper 40 % (far away / on a shelf): not in the corridor
        out, _ = decide(I(0, detections=[det("person", bottom=0.3)], tof_mm=NEAR), DecisionState())
        self.assertEqual(out.state, DRIVE)

    def test_corridor_edges(self):
        lo, hi = config.CORRIDOR_X
        for cx, exp in ((lo, STOP_OBSTACLE), (hi, STOP_OBSTACLE), (lo - 0.01, DRIVE), (hi + 0.01, DRIVE)):
            out, _ = decide(I(0, detections=[det(cx=cx)], tof_mm=NEAR), DecisionState())
            self.assertEqual(out.state, exp, f"cx={cx}")

    def test_person_far_caution(self):
        out, _ = decide(I(0, detections=[det("person", h=0.2)], tof_mm=MID), DecisionState())
        self.assertEqual(out.state, CAUTION)
        self.assertAlmostEqual(out.left, config.BASE_SPEED * config.CAUTION_FACTOR)
        self.assertAlmostEqual(out.right, config.BASE_SPEED * config.CAUTION_FACTOR)

    def test_medium_box_caution(self):
        out, _ = decide(I(0, detections=[det("car", h=0.4)], tof_mm=math.inf), DecisionState())
        self.assertEqual(out.state, CAUTION)

    def test_small_box_far_drives(self):
        out, _ = decide(I(0, detections=[det("person", h=0.2)], tof_mm=FAR), DecisionState())
        self.assertEqual(out.state, DRIVE)

    def test_tof_none_big_box_stops(self):
        out, _ = decide(I(0, detections=[det("person", h=0.6)], tof_mm=None), DecisionState())
        self.assertEqual(out.state, STOP_OBSTACLE)
        self.assertIn("no tof", out.reason)

    def test_tof_none_medium_box_caution(self):
        out, _ = decide(I(0, detections=[det("person", h=0.4)], tof_mm=None), DecisionState())
        self.assertEqual(out.state, CAUTION)

    def test_big_box_but_tof_clear_is_caution_not_stop(self):
        # vision fallback only applies when ToF has no reading
        out, _ = decide(I(0, detections=[det("person", h=0.6)], tof_mm=FAR), DecisionState())
        self.assertEqual(out.state, CAUTION)

    def test_traffic_classes_are_not_obstacles(self):
        for d in (red(h=0.9, cx=0.5), green(h=0.9, cx=0.5), sign(h=0.2)):
            d = dict(d, bottom_ratio=0.9)
            out, _ = decide(I(0, detections=[d], tof_mm=NEAR), DecisionState())
            self.assertNotEqual(out.state, STOP_OBSTACLE, d["cls"])


class TestSafetyNet(unittest.TestCase):
    def test_unknown_object_tof_150_two_readings_stops(self):
        outs, _ = run([I(0.0, tof_mm=150), I(0.1, tof_mm=150)])
        self.assertEqual(states(outs), [DRIVE, STOP_OBSTACLE])
        self.assertIn("safety", outs[1].reason)

    def test_single_close_reading_ignored(self):
        outs, _ = run([I(0.0, tof_mm=150), I(0.1, tof_mm=FAR), I(0.2, tof_mm=150), I(0.3, tof_mm=FAR)])
        self.assertEqual(states(outs), [DRIVE] * 4)

    def test_counts_readings_not_frames(self):
        # the same ToF reading seen on two frames is ONE reading
        outs, _ = run([I(1.00, tof_mm=150, tof_t=1.00), I(1.05, tof_mm=150, tof_t=1.00),
                       I(1.10, tof_mm=150, tof_t=1.07)])
        self.assertEqual(states(outs), [DRIVE, DRIVE, STOP_OBSTACLE])

    def test_both_ultrasonics_close_two_readings_stops(self):
        outs, _ = run([I(0.0, us_fl_mm=150, us_fr_mm=150), I(0.1, us_fl_mm=150, us_fr_mm=150)])
        self.assertEqual(states(outs), [DRIVE, STOP_OBSTACLE])

    def test_one_ultrasonic_close_ignored(self):
        outs, _ = run([I(t, us_fl_mm=150, us_fr_mm=900) for t in (0.0, 0.1, 0.2)])
        self.assertEqual(states(outs), [DRIVE] * 3)

    def test_safety_regardless_of_yolo(self):
        outs, _ = run([I(0.0, tof_mm=150, detections=[det("light_green")]),
                       I(0.1, tof_mm=150, detections=[det("light_green")])])
        self.assertEqual(outs[1].state, STOP_OBSTACLE)

    def test_safety_beats_red_and_sign(self):
        steps = [I(t, detections=[red(), sign()], tof_mm=150) for t in (0.0, 0.1, 0.2, 0.3)]
        outs, _ = run(steps)
        self.assertTrue(all(o.state == STOP_OBSTACLE for o in outs[1:]))


class TestFreshness(unittest.TestCase):
    def test_stale_detection_ignored(self):
        t = 10.0
        stale = t - config.DET_MAX_AGE_S - 0.01
        out, _ = decide(I(t, detections=[det(h=0.6)], det_t=stale, tof_mm=None), DecisionState())
        self.assertEqual(out.state, DRIVE)
        fresh = t - config.DET_MAX_AGE_S + 0.01
        out, _ = decide(I(t, detections=[det(h=0.6)], det_t=fresh, tof_mm=None), DecisionState())
        self.assertEqual(out.state, STOP_OBSTACLE)

    def test_stale_tof_treated_as_none(self):
        # person in corridor, ToF says 300 mm but the reading is 0.4 s old -> None -> vision fallback
        t = 5.0
        old = t - config.SENSOR_STALE_S - 0.1
        out, _ = decide(I(t, detections=[det(h=0.2)], tof_mm=NEAR, tof_t=old), DecisionState())
        self.assertEqual(out.state, DRIVE)
        out, _ = decide(I(t, detections=[det(h=0.6)], tof_mm=NEAR, tof_t=old), DecisionState())
        self.assertIn("no tof", out.reason)

    def test_stale_tof_does_not_feed_safety(self):
        outs, _ = run([I(1.0, tof_mm=150, tof_t=0.5), I(1.1, tof_mm=150, tof_t=0.6)])
        self.assertEqual(states(outs), [DRIVE, DRIVE])


class TestResumeHysteresis(unittest.TestCase):
    def test_resume_needs_clear_time_and_distance(self):
        steps = [I(0.0, detections=[det()], tof_mm=NEAR),       # stop
                 I(0.5, tof_mm=RESUME - 50),                     # person gone, 450 mm: clearing
                 I(CLEAR + 0.2, tof_mm=RESUME - 50),             # clear long enough, but 450 < 500
                 I(CLEAR + 0.3, tof_mm=RESUME + 100)]            # 600 > 500 and clear -> drive
        outs, _ = run(steps)
        self.assertEqual(states(outs), [STOP_OBSTACLE, STOP_OBSTACLE, STOP_OBSTACLE, DRIVE])
        self.assertIn("clearing", outs[1].reason)
        self.assertIn("waiting", outs[2].reason)

    def test_distance_ok_but_not_clear_long_enough(self):
        outs, _ = run([I(0.0, detections=[det()], tof_mm=NEAR), I(CLEAR * 0.5, tof_mm=FAR),
                       I(CLEAR * 0.99, tof_mm=FAR), I(CLEAR * 1.01, tof_mm=FAR)])
        self.assertEqual(states(outs), [STOP_OBSTACLE] * 3 + [DRIVE])

    def test_redetection_restarts_clear_timer(self):
        outs, _ = run([I(0.0, detections=[det()], tof_mm=NEAR), I(0.8, detections=[det()], tof_mm=NEAR),
                       I(1.5, tof_mm=FAR), I(1.81, tof_mm=FAR)])
        self.assertEqual(states(outs), [STOP_OBSTACLE] * 3 + [DRIVE])

    def test_no_target_counts_as_clear(self):
        outs, _ = run([I(0.0, detections=[det()], tof_mm=NEAR), I(CLEAR + 0.1, tof_mm=math.inf)])
        self.assertEqual(states(outs), [STOP_OBSTACLE, DRIVE])

    def test_tof_missing_uses_ultrasonics(self):
        outs, _ = run([I(0.0, detections=[det(h=0.6)]),                         # vision stop, no tof
                       I(CLEAR + 0.1, us_fl_mm=RESUME - 100, us_fr_mm=900),     # 400 < 500: wait
                       I(CLEAR + 0.2, us_fl_mm=RESUME + 100, us_fr_mm=900)])
        self.assertEqual(states(outs), [STOP_OBSTACLE, STOP_OBSTACLE, DRIVE])

    def test_no_distance_sensor_time_only(self):
        outs, _ = run([I(0.0, detections=[det(h=0.6)]), I(CLEAR * 0.9), I(CLEAR + 0.01)])
        self.assertEqual(states(outs), [STOP_OBSTACLE, STOP_OBSTACLE, DRIVE])

    def test_safety_stop_also_uses_hysteresis(self):
        outs, _ = run([I(0.0, tof_mm=150), I(0.1, tof_mm=150), I(0.6, tof_mm=RESUME - 10),
                       I(1.2, tof_mm=RESUME - 10), I(1.3, tof_mm=RESUME + 10)])
        self.assertEqual(states(outs), [DRIVE, STOP_OBSTACLE, STOP_OBSTACLE, STOP_OBSTACLE, DRIVE])


# ===========================================================================
# Traffic lights
# ===========================================================================
class TestRedLight(unittest.TestCase):
    def test_red_far_caution(self):
        far = config.RED_LIGHT_HEIGHT_RATIO - 0.05
        outs, _ = run([I(i * DT, detections=[red(h=far)]) for i in range(N_SIG)])
        self.assertEqual(states(outs), [DRIVE] * (N_SIG - 1) + [CAUTION])
        self.assertAlmostEqual(outs[-1].left, config.BASE_SPEED * config.CAUTION_FACTOR)

    def test_red_close_confirmed_stops(self):
        outs, _ = run([I(i * DT, detections=[red()]) for i in range(N_SIG)])
        self.assertEqual(states(outs), [DRIVE] * (N_SIG - 1) + [STOP_RED])

    def test_one_frame_red_flicker_ignored(self):
        steps = [I(i * DT, detections=[red()] if i % 2 == 0 else []) for i in range(8)]
        outs, _ = run(steps)
        self.assertEqual(states(outs), [DRIVE] * 8)

    def test_red_on_left_side_ignored(self):
        outs, _ = run([I(i * DT, detections=[red(cx=0.2)]) for i in range(5)])
        self.assertEqual(states(outs), [DRIVE] * 5)

    def test_low_conf_red_ignored(self):
        outs, _ = run([I(i * DT, detections=[red(conf=config.SIGNAL_CONF - 0.05)]) for i in range(5)])
        self.assertEqual(states(outs), [DRIVE] * 5)

    def test_green_confirmed_resumes(self):
        steps = [I(i * DT, detections=[red()]) for i in range(N_SIG)]
        steps += [I((N_SIG + i) * DT, detections=[green()]) for i in range(N_SIG)]
        outs, _ = run(steps)
        self.assertEqual(states(outs)[N_SIG - 1:], [STOP_RED] * N_SIG + [DRIVE])

    def test_green_flicker_does_not_resume(self):
        steps = [I(i * DT, detections=[red()]) for i in range(N_SIG)]
        steps += [I((N_SIG + i) * DT, detections=[green()] if i % 2 == 0 else []) for i in range(6)]
        outs, _ = run(steps)
        self.assertTrue(all(s == STOP_RED for s in states(outs)[N_SIG - 1:]))

    def test_red_and_green_together_is_red(self):
        # from DRIVE: red+green confirms red
        outs, _ = run([I(i * DT, detections=[red(), green()]) for i in range(N_SIG)])
        self.assertEqual(outs[-1].state, STOP_RED)
        # while stopped: red+green never counts as green
        steps = [I(i * DT, detections=[red()]) for i in range(N_SIG)]
        steps += [I((N_SIG + i) * DT, detections=[red(), green()]) for i in range(6)]
        outs, _ = run(steps)
        self.assertTrue(all(s == STOP_RED for s in states(outs)[N_SIG - 1:]))
        # red+green frames don't count towards the green confirmation: after red disappears,
        # N_SIG green-only checks are still needed
        t1 = (N_SIG + 6) * DT
        steps += [I(t1 + i * DT, detections=[green()]) for i in range(N_SIG)]
        outs, _ = run(steps)
        self.assertEqual(states(outs)[-N_SIG:], [STOP_RED] * (N_SIG - 1) + [DRIVE])

    def test_timeout_without_any_signal(self):
        steps = [I(i * DT, detections=[red()]) for i in range(N_SIG)]   # last red seen at t0
        t0 = (N_SIG - 1) * DT
        steps += [I(t0 + 5.0), I(t0 + config.RED_TIMEOUT_S - 0.05), I(t0 + config.RED_TIMEOUT_S)]
        outs, _ = run(steps)
        self.assertEqual(states(outs)[N_SIG - 1:], [STOP_RED, STOP_RED, STOP_RED, DRIVE])

    def test_red_seen_again_restarts_timeout(self):
        steps = [I(i * DT, detections=[red()]) for i in range(N_SIG)]
        t0 = (N_SIG - 1) * DT
        steps += [I(t0 + 8.0, detections=[red()]), I(t0 + config.RED_TIMEOUT_S + 1.0)]
        outs, _ = run(steps)
        self.assertEqual(outs[-1].state, STOP_RED)

    def test_obstacle_during_stop_red(self):
        steps = [I(i * DT, detections=[red()], tof_mm=FAR) for i in range(N_SIG)]
        t0 = N_SIG * DT
        steps += [I(t0, detections=[red(), det("person")], tof_mm=NEAR),
                  I(t0 + 0.5, detections=[red()], tof_mm=FAR),
                  I(t0 + CLEAR + 0.1, detections=[red()], tof_mm=FAR)]
        outs, _ = run(steps)
        self.assertEqual(states(outs)[N_SIG - 1:], [STOP_RED, STOP_OBSTACLE, STOP_OBSTACLE, STOP_RED])


class TestStopSign(unittest.TestCase):
    def test_needs_consecutive_checks(self):
        outs, _ = run([I(i * DT, detections=[sign()]) for i in range(N_SIGN)])
        self.assertEqual(states(outs), [DRIVE] * (N_SIGN - 1) + [STOP_SIGN])

    def test_flicker_ignored(self):
        outs, _ = run([I(i * DT, detections=[sign()] if i % 2 == 0 else []) for i in range(8)])
        self.assertEqual(states(outs), [DRIVE] * 8)

    def test_small_or_low_conf_sign_ignored(self):
        small = config.STOP_SIGN_HEIGHT_RATIO - 0.01
        outs, _ = run([I(i * DT, detections=[sign(h=small)]) for i in range(5)])
        self.assertEqual(states(outs), [DRIVE] * 5)
        outs, _ = run([I(i * DT, detections=[sign(conf=config.STOP_SIGN_CONF - 0.01)]) for i in range(5)])
        self.assertEqual(states(outs), [DRIVE] * 5)

    def test_height_at_threshold_counts(self):
        outs, _ = run([I(i * DT, detections=[sign(h=config.STOP_SIGN_HEIGHT_RATIO)]) for i in range(N_SIGN)])
        self.assertEqual(outs[-1].state, STOP_SIGN)

    def test_full_cycle_stop_ignore_stop_again(self):
        # sign permanently in view, one check every DT
        wait, ign = config.STOP_SIGN_WAIT_S, config.STOP_SIGN_IGNORE_S
        times = [round(i * DT, 3) for i in range(int((wait * 2 + ign + 1) / DT))]
        outs, _ = run([I(t, detections=[sign()]) for t in times])
        t_stop1 = (N_SIGN - 1) * DT
        by_t = dict(zip(times, states(outs)))
        self.assertEqual(by_t[round(t_stop1, 3)], STOP_SIGN)
        self.assertEqual(by_t[round(t_stop1 + wait - DT, 3)], STOP_SIGN)
        self.assertEqual(by_t[round(t_stop1 + wait, 3)], DRIVE)              # resume
        self.assertEqual(by_t[round(t_stop1 + wait + ign - DT, 3)], DRIVE)   # still ignoring
        self.assertEqual(by_t[round(t_stop1 + wait + ign, 3)], STOP_SIGN)    # active again

    def test_waits_full_time_even_if_sign_disappears(self):
        steps = [I(i * DT, detections=[sign()]) for i in range(N_SIGN)]
        t0 = (N_SIGN - 1) * DT
        steps += [I(t0 + 1.0), I(t0 + config.STOP_SIGN_WAIT_S - 0.01), I(t0 + config.STOP_SIGN_WAIT_S)]
        outs, _ = run(steps)
        self.assertEqual(states(outs)[N_SIGN - 1:], [STOP_SIGN, STOP_SIGN, STOP_SIGN, DRIVE])

    def test_red_beats_stop_sign(self):
        outs, _ = run([I(i * DT, detections=[red(), sign()]) for i in range(N_SIG)])
        self.assertEqual(outs[-1].state, STOP_RED)


class TestEstop(unittest.TestCase):
    def test_estop_overrides_everything(self):
        out, _ = decide(I(0, tof_mm=FAR, estop=True), DecisionState())
        self.assertEqual(out.state, ESTOP)
        out, _ = decide(I(0, None, detections=[det()], tof_mm=10, estop=True), DecisionState(lost_frames=99))
        self.assertEqual(out.state, ESTOP)
        self.assertEqual((out.left, out.right), (0.0, 0.0))

    def test_drive_after_estop_release(self):
        outs, _ = run([I(0.0, estop=True), I(0.1)])
        self.assertEqual(states(outs), [ESTOP, DRIVE])


class TestPurity(unittest.TestCase):
    def test_state_not_mutated(self):
        st = DecisionState()
        decide(I(0, 50.0, detections=[det()], tof_mm=NEAR), st)
        self.assertEqual(st, DecisionState())

    def test_deterministic(self):
        inp = I(1.0, 12.0, detections=[det("car", h=0.3)], tof_mm=900)
        st = DecisionState(prev_offset=10.0, prev_t=0.9)
        self.assertEqual(decide(inp, st), decide(inp, st))


if __name__ == "__main__":
    unittest.main()
