"""
Tests for the control loop, motors and sensor scheduling with fakes. No hardware.

    python test_control.py -v

  * an exception anywhere in the loop -> motors stopped, EXCEPTION row in the CSV, clean return
  * motors: stop() = brake (IN1=IN2=1, PWM 100) for BRAKE_S, then coast; watchdog still works
  * sensors: the two ultrasonics are triggered alternately, never together, >= US_GAP_S apart
"""
import csv
import io
import signal
import sys
import threading
import time
import types
import unittest

import numpy as np

import config
import main_demo


# ---------------------------------------------------------------------------
# fakes for the control loop
# ---------------------------------------------------------------------------
class FakeCam:
    def __init__(self, n=50, fail_at=None):
        self.n, self.i, self.fail_at = n, 0, fail_at

    def read(self):
        self.i += 1
        if self.fail_at is not None and self.i >= self.fail_at:
            raise RuntimeError("camera died")
        return None if self.i > self.n else np.zeros((config.CAM_HEIGHT, config.CAM_WIDTH, 3), np.uint8)


class FakeLane:
    def __init__(self, fail_at=None):
        self.i, self.fail_at = 0, fail_at

    def process(self, frame, draw=False):
        self.i += 1
        if self.fail_at is not None and self.i >= self.fail_at:
            raise ValueError("lane crashed")
        return 0.0, None


class FakeDetector:
    def __init__(self, fail_at=None):
        self.i, self.fail_at = 0, fail_at

    def detect(self, frame):
        self.i += 1
        if self.fail_at is not None and self.i >= self.fail_at:
            raise RuntimeError("NCNN inference failed")
        return []


class FakeSensors:
    def tof_reading(self):
        return 1500.0, time.monotonic()

    def ultrasonic_readings(self):
        return (1500.0, time.monotonic()), (1500.0, time.monotonic())

    def alive(self):
        return True


class RecordingMotors:
    """Tracks whether the wheels would be turning."""

    def __init__(self, fail_on_drive=None):
        self.calls = []
        self.moving = False
        self.fail_on_drive = fail_on_drive
        self.n_drive = 0

    def drive(self, left, right):
        self.n_drive += 1
        if self.fail_on_drive is not None and self.n_drive == self.fail_on_drive:
            raise OSError("lgpio write failed")
        self.calls.append(("drive", left, right))
        self.moving = left != 0 or right != 0

    def stop(self, block=False):
        self.calls.append(("stop",))
        self.moving = False


class Keys:
    def __init__(self, at, key):
        self.i, self.at, self.key = 0, at, key

    def get(self):
        self.i += 1
        return self.key if self.i == self.at else ""


def run_loop(**kw):
    buf = io.StringIO()
    log = csv.writer(buf)
    log.writerow(main_demo.CSV_HEADER)
    motors = kw.pop("motors", RecordingMotors())
    args = dict(cam=FakeCam(), lane=FakeLane(), detector=FakeDetector(), sensors=FakeSensors())
    args.update(kw)
    result = main_demo.control_loop(args["cam"], args["lane"], args["detector"], args["sensors"],
                                    motors, log, keys=args.get("keys"), max_frames=args.get("max_frames", 0))
    rows = list(csv.DictReader(io.StringIO(buf.getvalue())))
    return result, motors, rows


class TestControlLoopExceptions(unittest.TestCase):
    def assert_stopped_and_logged(self, result, motors, rows, text):
        self.assertEqual(result, "exception")
        self.assertIn(("stop",), motors.calls)
        self.assertFalse(motors.moving, "motors still running after exception")
        self.assertEqual(rows[-1]["state"], "EXCEPTION")
        self.assertIn(text, rows[-1]["reason"])
        self.assertTrue(any(r["state"] == "DRIVE" for r in rows[:-1]), "was driving before the error")

    def test_yolo_exception_stops_motors(self):
        self.assert_stopped_and_logged(*run_loop(detector=FakeDetector(fail_at=3)), "NCNN inference failed")

    def test_camera_exception_stops_motors(self):
        self.assert_stopped_and_logged(*run_loop(cam=FakeCam(fail_at=6)), "camera died")

    def test_lane_exception_stops_motors(self):
        self.assert_stopped_and_logged(*run_loop(lane=FakeLane(fail_at=5)), "lane crashed")

    def test_decision_exception_stops_motors(self):
        real = main_demo.decide
        calls = {"n": 0}

        def boom(inp, st):
            calls["n"] += 1
            if calls["n"] == 4:
                raise KeyError("bad state")
            return real(inp, st)
        main_demo.decide = boom
        try:
            self.assert_stopped_and_logged(*run_loop(), "bad state")
        finally:
            main_demo.decide = real

    def test_motor_write_exception_still_stops(self):
        result, motors, rows = run_loop(motors=RecordingMotors(fail_on_drive=4))
        self.assertEqual(result, "exception")
        self.assertIn(("stop",), motors.calls)
        self.assertFalse(motors.moving)

    def test_csv_has_ages_and_state(self):
        result, motors, rows = run_loop(max_frames=7)
        self.assertEqual(result, "max_frames")
        for col in ("det_age", "tof_age", "us_fl_age", "us_fr_age", "state", "reason"):
            self.assertIn(col, rows[0])
        self.assertNotEqual(rows[1]["det_age"], "-")
        self.assertLess(float(rows[1]["tof_age"]), 0.1)

    def test_q_and_end_of_source_leave_motors_stopped(self):
        for kw, expected in ((dict(keys=Keys(5, "q")), "quit"), (dict(cam=FakeCam(n=4)), "end")):
            result, motors, rows = run_loop(**kw)
            self.assertEqual(result, expected)
            self.assertFalse(motors.moving)


# ---------------------------------------------------------------------------
# fake lgpio for motors / sensors
# ---------------------------------------------------------------------------
class FakeLgpio(types.ModuleType):
    BOTH_EDGES = 3

    def __init__(self):
        super().__init__("lgpio")
        self.lock = threading.Lock()
        self.level = {}
        self.duty = {}
        self.log = []

    def gpiochip_open(self, chip):
        return 100 + chip

    def gpiochip_close(self, h):
        pass

    def gpio_claim_output(self, h, pin, level=0):
        self.level[pin] = level

    def gpio_free(self, h, pin):
        self.log.append(("free", pin))

    def gpio_write(self, h, pin, level):
        with self.lock:
            self.level[pin] = level
            self.log.append(("write", pin, level))

    def tx_pwm(self, h, pin, freq, duty):
        with self.lock:
            self.duty[pin] = duty
            self.log.append(("pwm", pin, duty))

    def ins(self):
        return tuple(self.level[p] for p in (config.LEFT_IN1, config.LEFT_IN2, config.RIGHT_IN1, config.RIGHT_IN2))

    def pwms(self):
        return self.duty[config.LEFT_PWM], self.duty[config.RIGHT_PWM]


class MotorTestBase(unittest.TestCase):
    def setUp(self):
        self.lg = FakeLgpio()
        self._saved = sys.modules.get("lgpio")
        sys.modules["lgpio"] = self.lg
        self._sigint = signal.getsignal(signal.SIGINT)
        self._sigterm = signal.getsignal(signal.SIGTERM)
        from motors import Motors
        self.m = Motors()

    def tearDown(self):
        self.m.cleanup()
        signal.signal(signal.SIGINT, self._sigint)
        signal.signal(signal.SIGTERM, self._sigterm)
        if self._saved is None:
            sys.modules.pop("lgpio", None)
        else:
            sys.modules["lgpio"] = self._saved


class TestMotorsBrake(MotorTestBase):
    def test_drive_forward(self):
        self.m.drive(30, 30)
        self.assertEqual(self.lg.pwms(), (30.0, 30.0))
        self.assertEqual(self.lg.ins(), (0, 1, 1, 0))   # left inverted, right not

    def test_stop_brakes_then_coasts(self):
        self.m.drive(30, 30)
        self.m.stop()
        self.assertEqual(self.lg.ins(), (1, 1, 1, 1), "IN1=IN2=1 = short brake")
        self.assertEqual(self.lg.pwms(), (100, 100))
        time.sleep(config.BRAKE_S * 0.5)
        self.assertEqual(self.lg.ins(), (1, 1, 1, 1), "still braking")
        time.sleep(config.BRAKE_S * 0.5 + 0.1)
        self.assertEqual(self.lg.ins(), (0, 0, 0, 0), "coast after BRAKE_S")
        self.assertEqual(self.lg.pwms(), (0, 0))

    def test_drive_zero_brakes(self):
        self.m.drive(30, 30)
        self.m.drive(0, 0)
        self.assertEqual(self.lg.ins(), (1, 1, 1, 1))
        for _ in range(int(config.BRAKE_S / 0.05) + 3):   # main loop keeps sending (0, 0)
            self.m.drive(0, 0)
            time.sleep(0.05)
        self.assertEqual(self.lg.ins(), (0, 0, 0, 0))

    def test_drive_after_brake_resumes(self):
        self.m.drive(30, 30)
        self.m.stop()
        self.m.drive(20, 20)
        self.assertEqual(self.lg.pwms(), (20.0, 20.0))
        time.sleep(config.BRAKE_S + 0.1)
        self.assertEqual(self.lg.pwms(), (20.0, 20.0), "the brake timer must not coast a driving car")

    def test_stop_block(self):
        self.m.drive(30, 30)
        t0 = time.monotonic()
        self.m.stop(block=True)
        self.assertGreaterEqual(time.monotonic() - t0, config.BRAKE_S - 0.01)
        self.assertEqual(self.lg.ins(), (0, 0, 0, 0))

    def test_watchdog_brakes_when_loop_hangs(self):
        self.m.drive(30, 30)
        time.sleep(config.MOTOR_WATCHDOG_S + 0.1)
        self.assertIn(self.lg.ins(), ((1, 1, 1, 1), (0, 0, 0, 0)))
        time.sleep(config.BRAKE_S + 0.1)
        self.assertEqual(self.lg.pwms(), (0, 0))

    def test_cleanup_brakes_coasts_then_frees(self):
        self.m.drive(30, 30)
        self.m.cleanup()
        writes = [e for e in self.lg.log if e[0] in ("write", "pwm", "free")]
        first_brake = next(i for i, e in enumerate(writes) if e == ("pwm", config.LEFT_PWM, 100))
        first_free = next(i for i, e in enumerate(writes) if e[0] == "free")
        coast = [i for i, e in enumerate(writes) if e == ("pwm", config.LEFT_PWM, 0)]
        self.assertTrue(any(first_brake < i < first_free for i in coast), "brake -> coast -> free")


class TestUltrasonicScheduling(unittest.TestCase):
    def test_alternate_never_together(self):
        from sensors import Sensors
        s = Sensors(use_tof=False, use_ultrasonic=False, use_gyro=False)
        events = []
        busy = threading.Lock()

        class FakeUS:
            def __init__(self, name):
                self.name = name

            def read_mm(self):
                self_ok = busy.acquire(blocking=False)
                events.append((self.name, time.monotonic(), self_ok))
                time.sleep(0.005)
                if self_ok:
                    busy.release()
                return 1000.0

            def close(self):
                pass
        s.us_fl, s.us_fr = FakeUS("fl"), FakeUS("fr")
        s.start()
        time.sleep(0.8)
        s.close()
        names = [e[0] for e in events]
        self.assertGreaterEqual(len(names), 6)
        self.assertTrue(all(a != b for a, b in zip(names, names[1:])), f"not alternating: {names}")
        self.assertTrue(all(e[2] for e in events), "two pings overlapped")
        gaps = [b[1] - a[1] for a, b in zip(events, events[1:])]
        self.assertGreaterEqual(min(gaps), config.US_GAP_S - 0.005)

    def test_alive_reflects_thread(self):
        from sensors import Sensors
        s = Sensors(use_tof=False, use_ultrasonic=False, use_gyro=False)
        self.assertFalse(s.alive())
        s.start()
        time.sleep(0.2)
        self.assertTrue(s.alive())
        s.close()
        self.assertFalse(s.alive())


if __name__ == "__main__":
    unittest.main()
