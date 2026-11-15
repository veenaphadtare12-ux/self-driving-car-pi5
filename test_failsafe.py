"""
Fail-safe tests (UFLD Part 4): simulate each failure on the threaded runtime with fakes and confirm the
(dry-run) car stops within the stated time. No hardware.

    python test_failsafe.py -v

  * lane / detection thread crash                -> STOP within one tick
  * lane / detection / camera output frozen      -> STOP within FAILSAFE_STALE_S (1 s) + one tick
  * UFLD below 2.5 fps over 3 s while driving    -> STOP, Hough offered; switching only while stopped
  * CPU 82 C -> half speed; 86 C -> STOP
  * under-voltage appears                        -> warning + CSV, keeps driving
  * heartbeat missing 1.5 s while driving        -> STOP; regular heartbeats keep it driving
  * a fail-safe STOP latches until START
"""
import csv
import os
import tempfile
import threading
import time
import unittest

import config
from failsafe import FailSafe
from runtime import CarRuntime
from test_runtime import FakeCam, FakeDet, FakeLane, FakeSensors, RecMotors

TICK = 1.0 / config.CONTROL_HZ


class FreezeLane(FakeLane):
    def __init__(self, backend="hough", **kw):
        super().__init__(**kw)
        self.backend = backend
        self.frozen = threading.Event()

    def process(self, frame, draw=False):
        while self.frozen.is_set():
            time.sleep(0.01)
        return super().process(frame, draw)


class FreezeDet(FakeDet):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.frozen = threading.Event()

    def detect(self, frame):
        while self.frozen.is_set():
            time.sleep(0.01)
        return super().detect(frame)


class FreezeCam(FakeCam):
    def __init__(self, **kw):
        super().__init__(**kw)
        self.frozen = threading.Event()

    def read(self):
        while self.frozen.is_set():
            time.sleep(0.01)
        return super().read()


class FakeMon:
    ident = 1                      # "already started": the runtime won't start it

    def __init__(self, temp=50.0, throttled=0):
        self.temp_c, self.throttled = temp, throttled
        self.temp_max, self.throttled_seen, self.rss_mb = temp, throttled, 100.0

    def stop(self):
        pass


class Harness:
    def __init__(self, cam=None, lane=None, det=None, mon=None, log_path=None, lane_factory=None):
        self.m = RecMotors()
        self.mon = mon or FakeMon()
        self.rt = CarRuntime(cam or FreezeCam(), lane or FreezeLane(), det or FreezeDet(), FakeSensors(), self.m,
                             running=True, monitor=self.mon, log_path=log_path, lane_factory=lane_factory,
                             lane_backend=getattr(lane, "backend", None))
        self.rt.failsafe = FailSafe()
        self.rt.start_threads()
        self.th = threading.Thread(target=self.rt.run, daemon=True)
        self.th.start()

    def wait_stop(self, timeout):
        """Seconds until running goes False (None if it didn't within timeout)."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if not self.rt.running:
                return time.monotonic() - t0
            time.sleep(0.005)
        return None

    def close(self):
        self.rt.stop()
        self.th.join(timeout=2)
        self.rt.close()


class TestFailSafe(unittest.TestCase):
    def setUp(self):
        self.h = None

    def tearDown(self):
        if self.h:
            self.h.close()

    def settle(self, s=1.0):
        time.sleep(s)
        self.assertTrue(self.h.rt.running, self.h.rt.stop_reason)
        self.assertEqual(self.h.rt.last["state"], "DRIVE")

    def assert_stopped(self, dt, limit, text):
        self.assertIsNotNone(dt, f"no stop ({text})")
        print(f"\n  [{self._testMethodName}] stopped {dt:.3f}s after the fault (limit {limit:.2f}s): "
              f"{self.h.rt.stop_reason}", end="")
        self.assertLessEqual(dt, limit, f"stopped after {dt:.2f}s > {limit:.2f}s")
        self.assertIn(text, self.h.rt.stop_reason)
        self.assertEqual(self.h.m.calls[-1], (0.0, 0.0))
        self.assertGreaterEqual(self.h.m.stops, 1)
        return dt

    def test_lane_crash(self):
        self.h = Harness(lane=FreezeLane(fail_at=25))
        dt = self.h.wait_stop(3.0)
        self.assertIsNotNone(dt)
        self.assertIn("lane thread crashed", self.h.rt.stop_reason)
        print(f"\n  [test_lane_crash] {self.h.rt.stop_reason}", end="")

    def test_detection_crash(self):
        self.h = Harness(det=FreezeDet(fail_at=10))
        self.assertIsNotNone(self.h.wait_stop(3.0))
        self.assertIn("detection thread crashed", self.h.rt.stop_reason)

    def test_lane_frozen(self):
        lane = FreezeLane()
        self.h = Harness(lane=lane)
        self.settle()
        lane.frozen.set()
        self.result = self.assert_stopped(self.h.wait_stop(3.0), config.FAILSAFE_STALE_S + 0.1 + 2 * TICK, "lane thread stale")
        lane.frozen.clear()

    def test_detection_frozen(self):
        det = FreezeDet()
        self.h = Harness(det=det)
        self.settle()
        det.frozen.set()
        self.assert_stopped(self.h.wait_stop(3.0), config.FAILSAFE_STALE_S + 0.15 + 2 * TICK, "detection thread stale")
        det.frozen.clear()

    def test_camera_frozen(self):
        cam = FreezeCam()
        self.h = Harness(cam=cam)
        self.settle()
        cam.frozen.set()
        self.assert_stopped(self.h.wait_stop(3.0), config.FAILSAFE_STALE_S + 2 * TICK, "camera thread stale")
        cam.frozen.clear()

    def test_ufld_slow_offers_hough(self):
        lane = FreezeLane(backend="ufld", delay=0.02, offset=0.0)
        made = []
        self.h = Harness(lane=lane, lane_factory=lambda b, m=None: made.append(b) or FreezeLane(backend=b))
        self.settle(config.FAILSAFE_UFLD_WINDOW_S + 0.5)        # healthy UFLD keeps driving past the window
        lane.delay = 0.6                                         # ~1.7 fps
        dt = self.h.wait_stop(8.0)
        self.assertIsNotNone(dt)
        print(f"\n  [test_ufld_slow] stopped {dt:.3f}s after UFLD slowed to ~1.7 fps: {self.h.rt.stop_reason}", end="")
        # rate over a 3 s window drops below 2.5 once fewer than 7.5 results are in the window
        self.assertLessEqual(dt, config.FAILSAFE_UFLD_WINDOW_S + 0.6 + 2 * TICK)
        self.assertIn("UFLD unhealthy", self.h.rt.stop_reason)
        self.assertTrue(self.h.rt.failsafe.offer_hough)
        self.h.rt.cmd_start()
        with self.assertRaises(RuntimeError):
            self.h.rt.set_lane("hough")                          # only while stopped
        self.h.rt.cmd_stop()
        self.h.rt.set_lane("hough")
        self.assertEqual(made, ["hough"])
        self.assertEqual(self.h.rt.lane_backend, "hough")
        self.assertFalse(self.h.rt.failsafe.offer_hough)

    def test_hot_half_speed_then_stop(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "r.csv")
            self.h = Harness(log_path=p)
            self.settle()
            full = self.h.m.calls[-1]
            self.h.mon.temp_c = 82.0
            time.sleep(0.3)
            half = self.h.m.calls[-1]
            self.assertTrue(self.h.rt.running)
            self.assertAlmostEqual(half[0], full[0] * config.FAILSAFE_HOT_FACTOR, places=6)
            self.assertAlmostEqual(half[1], full[1] * config.FAILSAFE_HOT_FACTOR, places=6)
            self.h.mon.temp_c = 86.0
            self.assert_stopped(self.h.wait_stop(1.0), 2 * TICK + 0.02, "CPU 86.0C")
            self.h.rt.log_f.flush()
            with open(p) as f:
                rows = list(csv.DictReader(f))
            self.assertTrue(any("half speed" in r["failsafe"] for r in rows))
            self.assertTrue(any(r["cpu_temp"] == "86.0" for r in rows))

    def test_undervoltage_warns_keeps_driving(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "r.csv")
            self.h = Harness(log_path=p)
            self.settle()
            self.h.mon.throttled = 0x50005
            time.sleep(0.3)
            self.assertTrue(self.h.rt.running)
            self.assertIn("UNDER-VOLTAGE", self.h.rt.message)
            self.h.rt.log_f.flush()
            with open(p) as f:
                rows = list(csv.DictReader(f))
            self.assertTrue(any("UNDER-VOLTAGE" in r["failsafe"] and r["throttled"] == "0x50005" for r in rows))

    def test_old_undervoltage_since_boot_is_not_new(self):
        self.h = Harness(mon=FakeMon(throttled=0x50000))
        time.sleep(0.4)
        self.assertNotIn("UNDER-VOLTAGE", self.h.rt.message)

    def test_heartbeat(self):
        self.h = Harness()
        fs = self.h.rt.failsafe
        fs.heartbeat_required = True
        self.h.rt.cmd_start()
        t_end = time.monotonic() + 2.5
        while time.monotonic() < t_end:                          # page alive: heartbeats every 0.5 s
            fs.heartbeat()
            time.sleep(0.5)
        self.assertTrue(self.h.rt.running)
        t_last = fs.last_heartbeat                               # page closed
        dt = self.h.wait_stop(4.0)
        self.assertIsNotNone(dt)
        since = time.monotonic() - t_last
        print(f"\n  [test_heartbeat] stopped {since:.3f}s after the last heartbeat", end="")
        self.assertLessEqual(since, config.FAILSAFE_HEARTBEAT_S + 2 * TICK + 0.05)
        self.assertIn("page heartbeat lost", self.h.rt.stop_reason)

    def test_stop_latches_until_start(self):
        det = FreezeDet()
        self.h = Harness(det=det)
        self.settle()
        det.frozen.set()
        self.assertIsNotNone(self.h.wait_stop(3.0))
        det.frozen.clear()
        time.sleep(0.5)                                          # thread recovered ...
        self.assertFalse(self.h.rt.running)                      # ... but the car stays stopped
        self.assertEqual(self.h.rt.last["state"], "STOPPED")
        self.h.rt.cmd_start()
        time.sleep(0.3)
        self.assertTrue(self.h.rt.running)
        self.assertEqual(self.h.rt.last["state"], "DRIVE")


if __name__ == "__main__":
    unittest.main()
