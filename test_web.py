"""
Demo page tests (UFLD Part 5): real HTTP server on a free port, runtime with fakes, DRY RUN. No hardware.

    python test_web.py -v

  * page loads; starts STOPPED + DRY RUN; buttons respond < 0.5 s and change the state
  * one controller: a second browser is view-only but can E-STOP; E-STOP turns DRY RUN on
  * heartbeat lost while driving (browser closed) -> STOP within 1.5 s
  * phase change stops the car; "ignored by phase" shown; lane switch only while stopped
  * RECORD writes videos/<phase>_<time>.mp4 + logs/<phase>_<time>.csv; MJPEG stream delivers JPEGs
  * DRY RUN cannot be switched off when locked (replay / --dry-run); settings capped; Save + rehearsal files
  * camera missing -> CAMERA NOT CONNECTED, car cannot drive
"""
import csv
import json
import os
import socket
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request

import cv2
import numpy as np

import config
import web
from failsafe import FailSafe
from runtime import CarRuntime
from test_failsafe import FakeMon
from test_runtime import FakeCam, FakeDet, FakeLane, FakeSensors, RecMotors


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class Page:
    def __init__(self, base):
        self.base, self.token = base, None

    def post(self, path, body=None):
        b = dict(body or {})
        b["token"] = self.token
        req = urllib.request.Request(self.base + path, json.dumps(b).encode(), {"Content-Type": "application/json"})
        t0 = time.perf_counter()
        try:
            with urllib.request.urlopen(req, timeout=3) as r:
                code, obj = r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            code, obj = e.code, json.loads(e.read())
        return code, obj, time.perf_counter() - t0

    def claim(self, force=False):
        code, obj, _ = self.post("/claim", {"force": force})
        if code == 200:
            self.token = obj["token"]
        return code

    def status(self):
        with urllib.request.urlopen(f"{self.base}/status?token={self.token or ''}", timeout=3) as r:
            return json.loads(r.read())


class WebTestBase(unittest.TestCase):
    cam_factory = FakeCam

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        d = self.tmp.name
        self._saved = (web.SETTINGS_PATH, web.REHEARSAL_PATH, web.VIDEO_DIR, config.LOG_DIR, config.BASE_SPEED)
        web.SETTINGS_PATH, web.REHEARSAL_PATH = os.path.join(d, "s.json"), os.path.join(d, "rh.csv")
        web.VIDEO_DIR, config.LOG_DIR = os.path.join(d, "videos"), os.path.join(d, "logs")
        self.m = RecMotors()
        self.made = []
        self.det = FakeDet()
        self.rt = CarRuntime(self.cam_factory(), FakeLane(), self.det, FakeSensors(), self.m, running=False,
                             monitor=FakeMon(), log_path=os.path.join(d, "logs", "run.csv"),
                             lane_factory=lambda b, m=None: self.made.append((b, m)) or FakeLane())
        self.rt.failsafe = FailSafe()
        self.rt.start_threads()
        self.port = free_port()
        self.srv = web.start(self.rt, self.port, real_motors_factory=None, host="127.0.0.1")
        self.loop = threading.Thread(target=self.rt.run, daemon=True)
        self.loop.start()
        self.base = f"http://127.0.0.1:{self.port}"
        self.a, self.b = Page(self.base), Page(self.base)
        time.sleep(0.3)

    def tearDown(self):
        self.srv.shutdown()
        self.rt.stop()
        self.loop.join(timeout=2)
        self.rt.close()
        (web.SETTINGS_PATH, web.REHEARSAL_PATH, web.VIDEO_DIR, config.LOG_DIR, config.BASE_SPEED) = self._saved
        self.tmp.cleanup()


class TestWeb(WebTestBase):
    def test_page_and_initial_state(self):
        with urllib.request.urlopen(self.base + "/", timeout=3) as r:
            html = r.read().decode()
        self.assertIn("EMERGENCY STOP", html)
        self.assertIn("viewport", html)
        s = self.a.status()
        self.assertEqual(s["state"], "STOPPED")
        self.assertTrue(s["dry_run"])
        self.assertFalse(s["running"])
        self.assertEqual(sorted(s["phases"]), [f"P{i}" for i in range(1, 8)])

    def test_buttons_fast_and_effective(self):
        self.assertEqual(self.a.claim(), 200)
        times = []
        for action, want in (("start", "DRIVE"), ("stop", "STOPPED"), ("start", "DRIVE"), ("estop", "ESTOP")):
            code, obj, dt = self.a.post("/cmd", {"action": action})
            self.assertEqual(code, 200, obj)
            times.append(dt)
            time.sleep(0.2)
            self.assertEqual(self.a.status()["state"], want, action)
        code, obj, dt = self.a.post("/phase", {"phase": "P2"})
        times.append(dt)
        self.assertEqual(self.a.status()["phase"], "P2")
        self.assertLess(max(times), 0.5, times)
        self.max_cmd_s = max(times)

    def test_one_controller_second_view_only(self):
        self.assertEqual(self.a.claim(), 200)
        self.assertEqual(self.b.claim(), 409)
        code, obj, _ = self.b.post("/cmd", {"action": "start"})
        self.assertEqual(code, 403)
        self.assertFalse(self.b.status()["controller"])
        self.assertTrue(self.a.status()["controller"])
        self.a.post("/cmd", {"action": "start"})
        time.sleep(0.2)
        self.assertEqual(self.b.claim(force=True), 409)          # no takeover while driving
        code, obj, _ = self.b.post("/cmd", {"action": "estop"})  # but E-STOP works from any browser
        self.assertEqual(code, 200)
        time.sleep(0.15)
        s = self.a.status()
        self.assertEqual(s["state"], "ESTOP")
        self.assertTrue(s["dry_run"])
        self.assertEqual(self.b.claim(force=True), 200)          # stopped: takeover allowed
        self.assertFalse(self.a.status()["controller"])

    def test_heartbeat_loss_stops(self):
        self.assertEqual(self.a.claim(), 200)
        self.a.post("/cmd", {"action": "start"})
        for _ in range(4):                                       # page open: heartbeats
            self.a.post("/heartbeat")
            time.sleep(0.5)
        self.assertTrue(self.rt.running)
        t_last = time.monotonic()
        self.a.post("/heartbeat")                                # browser closes after this
        while self.rt.running and time.monotonic() - t_last < 4:
            time.sleep(0.01)
        dt = time.monotonic() - t_last
        self.assertFalse(self.rt.running)
        self.assertLessEqual(dt, config.FAILSAFE_HEARTBEAT_S + 0.2)
        self.assertIn("heartbeat lost", self.a.status()["message"])
        self.hb_stop_s = dt

    def test_phase_stops_and_ignored(self):
        car = {"cls": "car", "conf": 0.9, "box": (200, 200, 440, 470), "box_height_ratio": 0.56,
               "cx_ratio": 0.5, "bottom_ratio": 0.98}
        self.det.dets = [car]
        self.a.claim()
        self.a.post("/phase", {"phase": "P1"})
        self.a.post("/cmd", {"action": "start"})
        time.sleep(0.4)
        s = self.a.status()
        self.assertEqual(s["state"], "DRIVE")
        self.assertEqual(s["ignored"], ["car"])
        self.a.post("/phase", {"phase": "P7"})
        time.sleep(0.2)
        s = self.a.status()
        self.assertFalse(s["running"])
        self.assertIn("phase -> P7", s["reason"])

    def test_lane_switch_only_stopped(self):
        self.a.claim()
        self.a.post("/cmd", {"action": "start"})
        code, obj, _ = self.a.post("/lane", {"choice": "ufld:v2_plus"})
        self.assertEqual(code, 409)
        self.a.post("/cmd", {"action": "stop"})
        code, obj, _ = self.a.post("/lane", {"choice": "ufld:v2_plus"})
        self.assertEqual(code, 200, obj)
        t0 = time.monotonic()
        while self.a.status()["switching"] and time.monotonic() - t0 < 3:
            time.sleep(0.05)
        self.assertEqual(self.made, [("ufld", "v2_plus")])
        self.assertEqual(self.rt.lane_backend, "ufld")

    def test_record_and_stream(self):
        self.a.claim()
        self.a.post("/phase", {"phase": "P5"})
        req = urllib.request.urlopen(self.base + "/stream", timeout=3)
        buf = b""
        t0 = time.monotonic()
        while buf.count(b"\xff\xd9") < 3 and time.monotonic() - t0 < 3:
            buf += req.read(4096)
        req.close()
        self.assertGreaterEqual(buf.count(b"\xff\xd8"), 3)
        self.a.post("/record", {"on": True})
        time.sleep(1.5)
        s = self.a.status()
        self.assertTrue(s["recording"])
        self.assertTrue(s["rec_path"].startswith("P5_") and s["rec_path"].endswith(".mp4"))
        self.assertTrue(s["rec_csv"].startswith("P5_"))
        self.a.post("/record", {"on": False})
        mp4 = os.path.join(web.VIDEO_DIR, s["rec_path"])
        cap = cv2.VideoCapture(mp4)
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        self.assertGreaterEqual(n, 8)
        with open(os.path.join(config.LOG_DIR, s["rec_csv"])) as f:
            rows = list(csv.DictReader(f))
        self.assertGreater(len(rows), 20)
        self.assertEqual(rows[-1]["phase"], "P5")

    def test_dry_locked_settings_rehearsal(self):
        self.a.claim()
        code, obj, _ = self.a.post("/dryrun", {"on": False})
        self.assertEqual(code, 409)
        self.assertTrue(self.rt.dry_run)
        code, _, _ = self.a.post("/params", {"base_speed": config.WEB_BASE_SPEED_MAX + 1})
        self.assertEqual(code, 400)
        code, _, _ = self.a.post("/params", {"base_speed": 20})
        self.assertEqual(code, 200)
        self.assertEqual(config.BASE_SPEED, 20)
        self.a.post("/save_params")
        with open(web.SETTINGS_PATH) as f:
            self.assertEqual(json.load(f)["BASE_SPEED"], 20)
        for r in ("PASS", "FAIL", "PASS", "PASS"):
            self.a.post("/rehearsal", {"phase": "P1", "result": r})
        rh = self.a.status()["rehearsal"]["P1"]
        self.assertEqual((rh["pass"], rh["fail"], rh["streak"]), (3, 1, 2))
        self.assertTrue(os.path.exists(web.REHEARSAL_PATH))

    def test_control_rate_with_page_open(self):
        req = urllib.request.urlopen(self.base + "/stream", timeout=3)
        stop = threading.Event()

        def reader():
            try:
                while not stop.is_set():
                    req.read(8192)
            except Exception:                                    # noqa: BLE001
                pass
        threading.Thread(target=reader, daemon=True).start()
        self.rt.reset_stats()
        time.sleep(3)
        s = self.rt.bench_summary()
        stop.set()
        req.close()
        self.assertAlmostEqual(s["ctrl_hz"], config.CONTROL_HZ, delta=0.5)


class NoFrames:
    def read(self):
        time.sleep(0.2)
        while True:
            time.sleep(0.5)


class TestNoCamera(WebTestBase):
    cam_factory = NoFrames

    def test_camera_missing(self):
        req = urllib.request.urlopen(self.base + "/stream", timeout=3)
        buf = b""
        t0 = time.monotonic()
        while buf.count(b"\xff\xd9") < 1 and time.monotonic() - t0 < 3:
            buf += req.read(4096)
        req.close()
        i, j = buf.index(b"\xff\xd8"), buf.index(b"\xff\xd9") + 2
        img = cv2.imdecode(np.frombuffer(buf[i:j], np.uint8), cv2.IMREAD_COLOR)
        self.assertGreater(img[:, :, 2].mean() - img[:, :, 0].mean(), 0.5)   # red banner on black
        self.assertFalse(self.a.status()["indicators"]["camera"])
        self.a.claim()
        self.a.post("/cmd", {"action": "start"})
        t0 = time.monotonic()
        while self.rt.running and time.monotonic() - t0 < 3:
            time.sleep(0.02)
        self.assertFalse(self.rt.running)
        self.assertIn("camera thread stale", self.rt.stop_reason)


if __name__ == "__main__":
    unittest.main()
