"""
Tests for the threaded runtime (UFLD Part 3). No hardware, no models.

    python test_runtime.py -v

  * LaneFeed: the fixed-rate loop gives EXACTLY the old per-frame steering and lane-lost timing
  * hold ticks keep the last steering; stale lane results count as lost every tick
  * phases filter detections ("ignored by phase"); the safety net works in every phase
  * workers take the newest frame, respect every_n, publish timestamps
  * fixed-rate loop holds its rate; stopped = motors 0 while decide() keeps running
  * a crashed lane / detection thread stops the car
  * ReplayCamera paces and loops; CSV header starts with main_demo's header
"""
import csv
import os
import tempfile
import threading
import time
import unittest

import cv2
import numpy as np

import config
import runtime
from decision import DecisionInputs, DecisionState, decide
from runtime import CarRuntime, LaneFeed, Result, Slot, Frame, Worker, filter_phase


def lane_result(offset, t_frame, seq, proc=0.2):
    return Result((offset, None, None), t_frame, t_frame + proc, proc, seq, seq)


class TestLaneFeed(unittest.TestCase):
    def run_both(self, offsets, caps, procs, hz=20.0):
        """Old: decide() per lane frame at its capture time. New: ticks at hz, result visible at t_done."""
        old, st = [], DecisionState()
        for off, c in zip(offsets, caps):
            out, st = decide(DecisionInputs(t=c, offset_px=off), st)
            old.append(out)
        feed, st, new = LaneFeed(max_age=10.0), DecisionState(), []
        results = [lane_result(o, c, i + 1, p) for i, (o, c, p) in enumerate(zip(offsets, caps, procs))]
        t, k, cur = caps[0], 0, None
        holds = []
        while k < len(results) or cur is None:
            while k < len(results) and results[k].t_done <= t:
                cur = results[k]
                k += 1
            if cur is not None:
                off, st_in, mode = feed.prepare(st, cur, t)
                out, st2 = decide(DecisionInputs(t=t, offset_px=off), st_in)
                st2, out = feed.finish(st, st2, out, mode)
                if mode == "new":
                    new.append(out)
                else:
                    holds.append((out, new[-1] if new else None))
                st = st2
            t += 1.0 / hz
        return old, new, holds

    def test_steering_identical_to_per_frame_loop(self):
        rng = np.random.default_rng(1)
        n = 60
        caps = np.cumsum(rng.uniform(0.15, 0.3, n)).tolist()
        offsets = (np.sin(np.arange(n) / 5.0) * 80 + rng.normal(0, 5, n)).tolist()
        procs = rng.uniform(0.05, 0.12, n).tolist()
        old, new, holds = self.run_both(offsets, caps, procs)
        self.assertEqual(len(old), len(new))
        for a, b in zip(old, new):
            self.assertEqual(a.state, b.state)
            self.assertAlmostEqual(a.steer, b.steer, places=9)
            self.assertAlmostEqual(a.left, b.left, places=9)
        self.assertGreater(len(holds), 50)
        for h, last in holds:                       # hold ticks repeat the last steering
            if last is not None and h.state == "DRIVE":
                self.assertAlmostEqual(h.steer, last.steer, places=9)
                self.assertEqual(h.reason, "lane hold")

    def test_lane_lost_counted_per_lane_frame(self):
        n = 30
        caps = [i * 0.25 for i in range(n)]
        offsets = [10.0] * 5 + [None] * 20 + [5.0] * 5
        old, new, _ = self.run_both(offsets, caps, [0.1] * n)
        self.assertEqual([o.state for o in old], [o.state for o in new])
        first_lost = [o.state for o in new].index("STOP_LANE_LOST")
        self.assertEqual(first_lost, 5 + config.LANE_LOST_FRAMES - 1)

    def test_stale_lane_counts_every_tick(self):
        feed, st = LaneFeed(max_age=0.5), DecisionState()
        r = lane_result(20.0, 0.0, 1, 0.1)
        states = []
        t = 0.1
        while t < 2.0:
            off, st_in, mode = feed.prepare(st, r, t)
            out, st2 = decide(DecisionInputs(t=t, offset_px=off), st_in)
            st, out = feed.finish(st, st2, out, mode)
            states.append((round(t, 2), mode, out.state))
            t += 0.05
        lost_t = next(t for t, m, s in states if s == "STOP_LANE_LOST")
        self.assertLessEqual(lost_t, 0.5 + config.LANE_LOST_FRAMES * 0.05 + 1e-6)
        self.assertTrue(all(s == "DRIVE" for t, m, s in states if t <= 0.5))


class TestPhases(unittest.TestCase):
    dets = [{"cls": "car", "conf": 0.9}, {"cls": "light_red", "conf": 0.9}, {"cls": "stop_sign", "conf": 0.9}]

    def test_filters(self):
        self.assertEqual(filter_phase(self.dets, "P1"), ([], ["car", "light_red", "stop_sign"]))
        k, ig = filter_phase(self.dets, "P2")
        self.assertEqual([d["cls"] for d in k], ["car"])
        self.assertEqual(ig, ["light_red", "stop_sign"])
        k, ig = filter_phase(self.dets, "P5")
        self.assertEqual([d["cls"] for d in k], ["light_red"])
        k, ig = filter_phase(self.dets, "P6")
        self.assertEqual([d["cls"] for d in k], ["stop_sign"])
        self.assertEqual(filter_phase(self.dets, "P7"), (self.dets, []))
        self.assertEqual(set(runtime.PHASES), {f"P{i}" for i in range(1, 8)})


# ---------------------------------------------------------------------------
# runtime with fakes
# ---------------------------------------------------------------------------
class FakeCam:
    def __init__(self, fps=30.0, n=None):
        self.fps, self.n, self.i = fps, n, 0

    def read(self):
        time.sleep(1.0 / self.fps)
        self.i += 1
        if self.n is not None and self.i > self.n:
            return None
        return np.full((config.CAM_HEIGHT, config.CAM_WIDTH, 3), self.i % 255, np.uint8)


class FakeLane:
    backend, source = "hough", "fake"

    def __init__(self, offset=0.0, delay=0.02, fail_at=None):
        self.offset, self.delay, self.fail_at, self.i = offset, delay, fail_at, 0
        self.result = None

    def process(self, frame, draw=False):
        self.i += 1
        if self.fail_at is not None and self.i >= self.fail_at:
            raise ValueError("lane crashed")
        time.sleep(self.delay)
        return self.offset, None


class FakeDet:
    def __init__(self, dets=None, delay=0.02, fail_at=None):
        self.dets, self.delay, self.fail_at, self.i = dets or [], delay, fail_at, 0
        self.frames = []

    def detect(self, frame):
        self.i += 1
        self.frames.append(int(frame[0, 0, 0]))
        if self.fail_at is not None and self.i >= self.fail_at:
            raise RuntimeError("NCNN failed")
        time.sleep(self.delay)
        return list(self.dets)


class FakeSensors:
    def __init__(self, tof=1500.0):
        self.tof = tof

    def tof_reading(self):
        return self.tof, time.monotonic()

    def ultrasonic_readings(self):
        return (1500.0, time.monotonic()), (1500.0, time.monotonic())

    def alive(self):
        return True


class RecMotors:
    def __init__(self):
        self.calls, self.stops = [], 0

    def drive(self, l, r):
        self.calls.append((l, r))

    def stop(self, block=False):
        self.stops += 1


def make_rt(**kw):
    m = kw.pop("motors", RecMotors())
    rt = CarRuntime(kw.pop("cam", FakeCam()), kw.pop("lane", FakeLane()), kw.pop("det", FakeDet()),
                    kw.pop("sensors", FakeSensors()), m, **kw)
    return rt, m


class TestRuntime(unittest.TestCase):
    def test_worker_newest_frame_every_n(self):
        src = Slot()
        seen = []
        w = Worker("w", lambda fr: seen.append(fr.seq) or fr.seq, src, every_n=3)
        w.start()
        for i in range(1, 31):
            src.put(Frame(None, time.monotonic(), i))
            time.sleep(0.01)
        time.sleep(0.1)
        w.stop()
        self.assertTrue(seen)
        self.assertTrue(all(b - a >= 3 for a, b in zip(seen, seen[1:])), seen)
        _, r = w.out.get()
        self.assertEqual(r.payload, seen[-1])
        self.assertLessEqual(r.t_frame, r.t_done)

    def test_fixed_rate_and_dry(self):
        rt, m = make_rt(running=True, hz=20)
        rt.start_threads()
        try:
            res = rt.run(duration=2.0)
        finally:
            rt.close()
        self.assertEqual(res, "duration")
        s = rt.bench_summary()
        self.assertAlmostEqual(s["ctrl_hz"], 20.0, delta=0.5)
        self.assertLess(s["period_ms"]["max"], 50 + 15)
        self.assertGreater(s["lane_fps"], 10)
        self.assertGreater(s["det_fps"], 5)
        self.assertEqual(rt.last["state"], "DRIVE")
        self.assertIn((0, 0), m.calls[-1:])               # loop exit drives 0

    def test_stopped_drives_zero_but_decides(self):
        rt, m = make_rt(running=False, hz=20)
        rt.start_threads()
        try:
            rt.run(duration=1.0)
        finally:
            rt.close()
        self.assertTrue(all(c == (0.0, 0.0) or c == (0, 0) for c in m.calls))
        self.assertEqual(rt.last["state"], "STOPPED")
        self.assertEqual(rt.last["decision"], "DRIVE")
        self.assertIn("would: DRIVE", rt.last["reason"])

    def test_lane_crash_stops(self):
        rt, m = make_rt(lane=FakeLane(fail_at=10), running=True)
        rt.start_threads()
        try:
            rt.run(duration=1.5)
        finally:
            rt.close()
        self.assertFalse(rt.running)
        self.assertIn("lane thread crashed", rt.stop_reason)
        self.assertGreaterEqual(m.stops, 1)
        self.assertEqual(m.calls[-1], (0, 0))

    def test_detection_crash_stops(self):
        rt, m = make_rt(det=FakeDet(fail_at=3), running=True)
        rt.start_threads()
        try:
            rt.run(duration=1.0)
        finally:
            rt.close()
        self.assertFalse(rt.running)
        self.assertIn("detection thread crashed", rt.stop_reason)

    def test_safety_net_in_every_phase(self):
        for ph in runtime.PHASES:
            rt, m = make_rt(sensors=FakeSensors(tof=120.0), running=True, phase=ph)
            rt.start_threads()
            try:
                rt.run(duration=0.6)
            finally:
                rt.close()
            self.assertEqual(rt.last["state"], "STOP_OBSTACLE", ph)

    def test_phase_ignores_and_logs(self):
        car = {"cls": "car", "conf": 0.9, "box": (200, 200, 440, 470), "box_height_ratio": 0.56,
               "cx_ratio": 0.5, "bottom_ratio": 0.98}
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "run.csv")
            rt, m = make_rt(det=FakeDet([car]), sensors=FakeSensors(tof=300.0), running=True, phase="P1",
                            log_path=p)
            rt.start_threads()
            try:
                rt.run(duration=0.8)
            finally:
                rt.close()
            self.assertEqual(rt.last["state"], "DRIVE")           # car ignored in P1
            with open(p) as f:
                rows = list(csv.DictReader(f))
            self.assertTrue(any(r["ignored"] == "car" for r in rows))
            self.assertEqual(rows[-1]["phase"], "P1")
            rt, m = make_rt(det=FakeDet([car]), sensors=FakeSensors(tof=300.0), running=True, phase="P2")
            rt.start_threads()
            try:
                rt.run(duration=0.8)
            finally:
                rt.close()
            self.assertEqual(rt.last["state"], "STOP_OBSTACLE")

    def test_commands(self):
        rt, m = make_rt(running=False)
        rt.cmd_start()
        self.assertTrue(rt.running)
        rt.cmd_estop()
        self.assertTrue(rt.estop and not rt.running and rt.dry_run)
        self.assertGreaterEqual(m.stops, 1)
        with self.assertRaises(RuntimeError):
            rt.set_dry_run(False)                                  # locked: no real-motor factory
        rt.cmd_start()
        rt.set_phase("P3")
        self.assertFalse(rt.running)
        self.assertEqual(rt.phase, "P3")
        rt.close()

    def test_csv_header(self):
        import main_demo
        self.assertEqual(runtime.CSV_HEADER[:len(main_demo.CSV_HEADER)], main_demo.CSV_HEADER)


class TestReplay(unittest.TestCase):
    def test_folder_pacing_and_loop(self):
        from camera import ReplayCamera
        with tempfile.TemporaryDirectory() as d:
            for i in range(5):
                cv2.imwrite(os.path.join(d, f"f{i}.jpg"), np.full((100, 160, 3), i * 40, np.uint8))
            cam = ReplayCamera(d, fps=50, loop=True)
            t0 = time.monotonic()
            frames = [cam.read() for _ in range(12)]
            dt = time.monotonic() - t0
            self.assertEqual(frames[0].shape, (config.CAM_HEIGHT, config.CAM_WIDTH, 3))
            self.assertAlmostEqual(dt, 11 / 50, delta=0.05)
            self.assertEqual(cam.loops, 2)
            cam2 = ReplayCamera(d, fps=1000, loop=False)
            got = [cam2.read() for _ in range(6)]
            self.assertIsNone(got[-1])


if __name__ == "__main__":
    unittest.main()
