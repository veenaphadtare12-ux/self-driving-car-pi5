"""
Threaded runtime (UFLD Part 3): all inference outside the control loop.

  camera thread    keeps only the newest frame + its capture time (Slot)
  lane thread      lane backend (UFLD / Hough) on the newest frame -> LaneResult + offset, with timestamps
  detection thread YOLO NCNN on the newest frame, at most every RT_YOLO_EVERY_N-th camera frame
  control loop     fixed rate (CONTROL_HZ), in the caller's thread: latest results + sensors -> decide() -> motors

decide() and its rules are unchanged. The old loop called decide() once per camera frame, and two
things in decide() count per call: the lane-lost frame counter (LANE_LOST_FRAMES) and the steering
derivative (dt between calls). LaneFeed keeps those per LANE RESULT, exactly as before:
  * new lane result  -> decide() sees the offset; dt = time between the two frames' capture times
  * no new result    -> "hold" tick: decide() runs (E-STOP, safety net, obstacles, lights, signs are
                        evaluated at the full control rate) but the lane counter and the derivative
                        memory are left as they were and steering = the last steering
  * lane result older than LANE_MAX_AGE_S -> no lane (counted every tick -> STOP_LANE_LOST)
Detections already carry the capture time of the frame YOLO ran on (det_t); decide() counts its
confirmations per new det_t and expires them after DET_MAX_AGE_S, as before.

Phases (demo page): in P1-P6 detections of classes outside the phase are removed before decide()
and logged as "ignored by phase"; the safety net (ToF/ultrasonic), lane-lost stop and E-STOP are
always on because they don't depend on detections.

The car starts STOPPED unless running=True; motors get (0, 0) while stopped, but decide() keeps running
so the page/CSV show what the car would do ("would: DRIVE ...").
"""
import collections
import csv
import os
import subprocess
import threading
import time
import traceback
from dataclasses import dataclass, replace
from typing import Any, Optional

import config
from decision import DecisionInputs, DecisionState, decide

PHASES = {
    "P1": ("Lane following", ()),
    "P2": ("Obstacle in lane: stop, resume", tuple(config.OBSTACLE_CLASSES)),
    "P3": ("Obstacle beside lane: no stop", tuple(config.OBSTACLE_CLASSES)),
    "P4": ("Unknown object: safety net", ()),
    "P5": ("Traffic light: red stops, green goes", (config.RED_LIGHT_CLASS, config.GREEN_LIGHT_CLASS)),
    "P6": ("Stop sign: stop 3 s, continue", (config.STOP_SIGN_CLASS,)),
    "P7": ("FULL DEMO: all rules", None),          # None = every class
}

BASE_CSV_HEADER = ["time", "frame", "offset", "detections", "det_age", "tof", "tof_age",
                   "us_fl", "us_fl_age", "us_fr", "us_fr_age", "sensors_alive",
                   "state", "reason", "left", "right", "fps"]          # = main_demo.CSV_HEADER
EXTRA_CSV_HEADER = ["lane_source", "lane_fps", "det_fps", "lane_age", "det_age_lat", "cpu_temp", "throttled",
                    "phase", "running", "dry_run", "decision", "ignored", "failsafe", "tick_ms", "lane_seq"]
CSV_HEADER = BASE_CSV_HEADER + EXTRA_CSV_HEADER


def _f(v, spec=".0f"):
    return "-" if v is None else format(v, spec)


# ---------------------------------------------------------------------------
# plumbing
# ---------------------------------------------------------------------------
class Slot:
    """Newest value + sequence number; readers never block the writer."""

    def __init__(self):
        self._cv = threading.Condition()
        self._v = None
        self._seq = 0

    def put(self, v):
        with self._cv:
            self._v = v
            self._seq += 1
            self._cv.notify_all()

    def get(self):
        with self._cv:
            return self._seq, self._v

    def clear(self):
        with self._cv:
            self._v = None

    def wait_newer(self, seq, timeout):
        with self._cv:
            self._cv.wait_for(lambda: self._seq > seq, timeout)
            return self._seq, self._v


@dataclass
class Frame:
    img: Any
    t: float          # capture time (monotonic)
    seq: int


@dataclass
class Result:
    payload: Any
    t_frame: float    # capture time of the frame it was computed from
    t_done: float
    proc: float       # seconds of processing
    frame_seq: int
    seq: int = 0      # result counter


class RateMeter:
    """Events per second over the last `window` seconds."""

    def __init__(self, window=3.0, clock=time.monotonic):
        self.window, self.clock = window, clock
        self.ts = collections.deque()
        self.t0 = None
        self.lock = threading.Lock()

    def tick(self, t=None):
        t = self.clock() if t is None else t
        with self.lock:
            if self.t0 is None:
                self.t0 = t
            self.ts.append(t)
            while self.ts and self.ts[0] < t - self.window:
                self.ts.popleft()

    def rate(self, now=None):
        now = self.clock() if now is None else now
        with self.lock:
            while self.ts and self.ts[0] < now - self.window:
                self.ts.popleft()
            if self.t0 is None:
                return 0.0
            span = min(self.window, now - self.t0)
            return len(self.ts) / span if span > 0 else 0.0


class Agg:
    """Bounded running statistics (count/mean/max + last N values for percentiles)."""

    def __init__(self, keep=20000):
        self.n, self.sum, self.max, self.min = 0, 0.0, None, None
        self.vals = collections.deque(maxlen=keep)

    def add(self, v):
        if v is None:
            return
        self.n += 1
        self.sum += v
        self.max = v if self.max is None else max(self.max, v)
        self.min = v if self.min is None else min(self.min, v)
        self.vals.append(v)

    def mean(self):
        return self.sum / self.n if self.n else None

    def pct(self, p):
        if not self.vals:
            return None
        s = sorted(self.vals)
        return s[min(len(s) - 1, int(round(p / 100.0 * (len(s) - 1))))]

    def summary(self, scale=1.0, nd=3):
        r = lambda v: None if v is None else round(v * scale, nd)
        return {"n": self.n, "mean": r(self.mean()), "p99": r(self.pct(99)), "max": r(self.max), "min": r(self.min)}


# ---------------------------------------------------------------------------
# threads
# ---------------------------------------------------------------------------
class CameraThread(threading.Thread):
    def __init__(self, cam, slot=None, clock=time.monotonic):
        super().__init__(name="camera", daemon=True)
        self.cam, self.clock = cam, clock
        self.slot = slot or Slot()
        self.meter = RateMeter(clock=clock)
        self.seq = 0
        self.error = None
        self.ended = False
        self.last_t = None
        self._stop_ev = threading.Event()

    def run(self):
        try:
            while not self._stop_ev.is_set():
                img = self.cam.read()
                if img is None:
                    self.ended = True
                    return
                t = self.clock()
                self.seq += 1
                self.slot.put(Frame(img, t, self.seq))
                self.last_t = t
                self.meter.tick(t)
        except Exception as e:                                   # noqa: BLE001
            self.error = f"{type(e).__name__}: {e}"
            traceback.print_exc()

    def stop(self):
        self._stop_ev.set()


class Worker(threading.Thread):
    """Runs fn(frame) on the newest camera frame, at most every `every_n`-th frame; publishes Result."""

    def __init__(self, name, fn, src, every_n=1, clock=time.monotonic):
        super().__init__(name=name, daemon=True)
        self.fn, self.src, self.every_n, self.clock = fn, src, max(1, int(every_n)), clock
        self.out = Slot()
        self.meter = RateMeter(clock=clock)
        self.proc = Agg(keep=2000)
        self.error = None
        self.last_publish = None
        self.started_t = None
        self.n = 0
        self._stop_ev = threading.Event()

    def run(self):
        self.started_t = self.clock()
        last = 0
        try:
            while not self._stop_ev.is_set():
                seq, fr = self.src.wait_newer(last + self.every_n - 1, 0.2)
                if fr is None or fr.seq < last + self.every_n:
                    continue
                last = fr.seq
                t0 = self.clock()
                payload = self.fn(fr)
                t1 = self.clock()
                if self._stop_ev.is_set():
                    return
                self.n += 1
                self.out.put(Result(payload, fr.t, t1, t1 - t0, fr.seq, self.n))
                self.last_publish = t1
                self.proc.add(t1 - t0)
                self.meter.tick(t1)
        except Exception as e:                                   # noqa: BLE001
            self.error = f"{type(e).__name__}: {e}"
            traceback.print_exc()

    def stop(self):
        self._stop_ev.set()


def read_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:                                            # noqa: BLE001
        return None


def read_throttled():
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True, timeout=2).stdout
        return int(out.strip().split("=")[-1], 16)
    except Exception:                                            # noqa: BLE001
        return None


def read_rss_mb():
    try:
        with open("/proc/self/statm") as f:
            return int(f.read().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 1e6
    except Exception:                                            # noqa: BLE001
        return None


class SysMonitor(threading.Thread):
    """CPU temperature (1 s), throttled flags (2 s), process memory. Fakeable: set .temp_c / .throttled."""

    def __init__(self, period=1.0, temp_fn=read_cpu_temp, throttled_fn=read_throttled):
        super().__init__(name="sysmon", daemon=True)
        self.period, self.temp_fn, self.throttled_fn = period, temp_fn, throttled_fn
        self.temp_c = temp_fn() if temp_fn else None
        self.throttled = throttled_fn() if throttled_fn else None
        self.throttled_seen = self.throttled or 0     # OR of every value seen while running
        self.rss_mb = read_rss_mb()
        self.temp_max = self.temp_c
        self._stop_ev = threading.Event()

    def run(self):
        k = 0
        while not self._stop_ev.wait(self.period):
            if self.temp_fn:
                self.temp_c = self.temp_fn()
                if self.temp_c is not None:
                    self.temp_max = self.temp_c if self.temp_max is None else max(self.temp_max, self.temp_c)
            k += 1
            if self.throttled_fn and k % 2 == 0:
                th = self.throttled_fn()
                if th is not None:
                    self.throttled = th
                    self.throttled_seen |= th
            self.rss_mb = read_rss_mb()

    def stop(self):
        self._stop_ev.set()


# ---------------------------------------------------------------------------
# lane results -> decide(), per lane result (see module docstring)
# ---------------------------------------------------------------------------
class LaneFeed:
    def __init__(self, max_age=None):
        self.max_age = config.LANE_MAX_AGE_S if max_age is None else max_age
        self.last_seq = None
        self.prev_cap = None

    def reset(self):
        self.last_seq = self.prev_cap = None

    def prepare(self, st, lane, now):
        """Returns (offset for decide, state for decide, mode); mode is 'new', 'hold' or 'stale'."""
        if lane is None or now - lane.t_frame > self.max_age:
            return None, st, "stale"
        if lane.seq != self.last_seq:
            st_in = st
            if st.prev_t is not None and self.prev_cap is not None:
                # derivative over the capture interval of the two lane frames, not the tick interval
                st_in = replace(st, prev_t=now - (lane.t_frame - self.prev_cap))
            self.last_seq, self.prev_cap = lane.seq, lane.t_frame
            return lane.payload[0], st_in, "new"
        # hold: decide(offset None) with the counter pre-decremented so it comes back unchanged
        return None, replace(st, lost_frames=st.lost_frames - 1), "hold"

    @staticmethod
    def finish(st_before, st_after, out, mode):
        if mode != "hold":
            return st_after, out
        st_after = replace(st_after, lost_frames=st_before.lost_frames)
        if st_after.prev_offset is None and out.state in ("DRIVE", "CAUTION"):
            st_after = replace(st_after, prev_offset=st_before.prev_offset, prev_t=st_before.prev_t)
            if out.reason.startswith("dropout"):
                out = replace(out, reason="lane hold" if st_before.lost_frames == 0
                              else f"dropout {st_before.lost_frames}")
        return st_after, out


def filter_phase(dets, phase):
    allowed = PHASES[phase][1]
    if allowed is None:
        return list(dets), []
    keep = [d for d in dets if d.get("cls") in allowed]
    ignored = sorted({d.get("cls") for d in dets if d.get("cls") not in allowed})
    return keep, ignored


# ---------------------------------------------------------------------------
# runtime
# ---------------------------------------------------------------------------
class CarRuntime:
    """Owns the threads and runs the fixed-rate control loop.

    lane / detector: objects with .process(frame, draw) and .detect(frame) (tests pass fakes);
    lane_factory(backend, model) builds a new lane object when the backend is switched (while stopped).
    """

    def __init__(self, cam, lane, detector=None, sensors=None, motors=None, *, lane_backend=None,
                 lane_factory=None, hz=None, yolo_every_n=None, phase="P7", running=True,
                 log_path=None, monitor=None, clock=time.monotonic, verbose=False, steer_cfg=None):
        from lanes import steer_cfg as lane_steer_cfg
        from motors import DryRunMotors
        self.clock = clock
        self.hz = hz or config.CONTROL_HZ
        self.period = 1.0 / self.hz
        self.cam_thread = CameraThread(cam, clock=clock)
        self.cam = cam
        self.lane = lane
        self.lane_backend = lane_backend or getattr(lane, "backend", "hough")
        self.lane_factory = lane_factory
        self.steer_cfg = steer_cfg if steer_cfg is not None else lane_steer_cfg(self.lane_backend)
        self.detector = detector
        self.yolo_every_n = yolo_every_n or config.RT_YOLO_EVERY_N
        self.sensors = sensors
        self.dry_motors = motors or DryRunMotors(verbose=False)
        self.motors = self.dry_motors
        self.dry_run = True
        self.monitor = monitor
        self.verbose = verbose

        self.lane_worker = self._make_lane_worker(lane)
        self.det_worker = self._make_det_worker(detector) if detector is not None else None

        self.st = DecisionState()
        self.feed = LaneFeed()
        self.phase = phase
        self.running = running
        self.estop = False
        self.message = ""
        self.stop_reason = ""
        self.failsafe = None            # Part 4 hook: object with check(rt, now) -> (action, reason)
        self.lock = threading.RLock()
        self.last = {}
        self.history = collections.deque(maxlen=10)
        self._last_state = None
        self.tick_i = 0
        self.t_start = None
        self._stop_ev = threading.Event()
        self.on_tick = []               # callbacks(rt, info) after every tick (web view, recorder)

        # stats
        self.tick_meter = RateMeter(clock=clock)
        self.s_period = Agg()
        self.s_lane_age = Agg()
        self.s_det_age = Agg()
        self.s_lane_lat = Agg()
        self.s_det_lat = Agg()
        self.s_tick_ms = Agg()
        self.overruns = 0
        self._prev_tick = None

        self.log_f = self.log = None
        self.log_path = log_path
        if log_path:
            self.open_log(log_path)

    # -- construction helpers -------------------------------------------------
    def _make_lane_worker(self, lane):
        return Worker("lane", lambda fr: lane.process(fr.img, draw=False) + (getattr(lane, "result", None),),
                      self.cam_thread.slot, 1, self.clock)

    def _make_det_worker(self, det):
        return Worker("detect", lambda fr: det.detect(fr.img), self.cam_thread.slot, self.yolo_every_n, self.clock)

    def open_log(self, path):
        self.close_log()
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self.log_f = open(path, "w", newline="")
        self.log = csv.writer(self.log_f)
        self.log.writerow(CSV_HEADER)
        self.log_path = path

    def close_log(self):
        if self.log_f is not None:
            try:
                self.log_f.close()
            except Exception:                                    # noqa: BLE001
                pass
        self.log_f = self.log = None

    def start_threads(self):
        self.cam_thread.start()
        self.lane_worker.start()
        if self.det_worker is not None:
            self.det_worker.start()
        if self.monitor is not None and self.monitor.ident is None:
            self.monitor.start()
            self._own_monitor = True

    def stop_threads(self):
        for t in (self.lane_worker, self.det_worker, self.cam_thread,
                  self.monitor if getattr(self, "_own_monitor", False) else None):
            if t is not None:
                t.stop()

    def reset_stats(self):
        with self.lock:
            self.s_period, self.s_lane_age, self.s_det_age = Agg(), Agg(), Agg()
            self.s_lane_lat, self.s_det_lat, self.s_tick_ms = Agg(), Agg(), Agg()
            self.overruns = 0
            self._prev_tick = None

    # -- commands (web / keys); all take effect immediately -------------------
    def cmd_start(self):
        with self.lock:
            if self.estop:
                self.estop = False
            self.running = True
            self.stop_reason = ""
            self.message = ""
            if self.failsafe is not None:
                self.failsafe.on_start(self)

    def cmd_stop(self, reason="STOP"):
        with self.lock:
            self.running = False
            self.stop_reason = reason
            self.motors.stop()

    def cmd_estop(self, reason="EMERGENCY STOP"):
        with self.lock:
            self.estop = True
            self.running = False
            self.stop_reason = reason
            self.motors.stop()
            self.set_dry_run(True)

    def set_dry_run(self, on, real_motors_factory=None):
        with self.lock:
            if on:
                if self.motors is not self.dry_motors:
                    self.motors.stop()
                self.motors, self.dry_run = self.dry_motors, True
                return True
            if self.running:
                raise RuntimeError("stop the car before switching DRY RUN off")
            if real_motors_factory is None:
                raise RuntimeError("DRY RUN is locked on (replay source or --dry-run)")
            self.motors, self.dry_run = real_motors_factory(), False
            return True

    def set_phase(self, phase):
        if phase not in PHASES:
            raise ValueError(f"unknown phase {phase!r}")
        with self.lock:
            self.cmd_stop(f"phase -> {phase}")
            self.phase = phase
            # fresh decision state per phase (the car is stopped): no latch from the previous phase's rules,
            # e.g. an obstacle stop seen in P7 must not hold the car for OBSTACLE_CLEAR_S after switching to P1
            self.st = DecisionState()
            self.feed.reset()

    def set_lane(self, backend, model=None):
        """Switch lane backend (only while stopped). Blocks while the new model loads."""
        from lanes import steer_cfg as lane_steer_cfg
        with self.lock:
            if self.running:
                raise RuntimeError("stop the car before changing the lane backend")
            if self.lane_factory is None:
                raise RuntimeError("no lane factory")
        new_lane = self.lane_factory(backend, model)
        with self.lock:
            old = self.lane_worker
            old.stop()
            self.lane = new_lane
            self.lane_backend = backend
            self.steer_cfg = lane_steer_cfg(backend)
            self.lane_worker = self._make_lane_worker(new_lane)
            self.feed.reset()
            self.st = DecisionState()
            self.lane_worker.start()
            if self.failsafe is not None:
                self.failsafe.on_lane_switch(self)

    # -- one control tick -----------------------------------------------------
    def tick(self, now=None):
        now = self.clock() if now is None else now
        t_in = time.perf_counter()
        if self.t_start is None:
            self.t_start = now
        with self.lock:
            _, lane = self.lane_worker.out.get()
            det = None
            if self.det_worker is not None:
                _, det = self.det_worker.out.get()

            offset, st_in, mode = self.feed.prepare(self.st, lane, now)
            dets_raw, det_t = (det.payload, det.t_frame) if det is not None else ([], None)
            dets, ignored = filter_phase(dets_raw, self.phase)

            if self.sensors is not None:
                tof, tof_t = self.sensors.tof_reading()
                (fl, fl_t), (fr, fr_t) = self.sensors.ultrasonic_readings()
                alive = self.sensors.alive()
            else:
                tof = tof_t = fl = fl_t = fr = fr_t = None
                alive = "-"

            inp = DecisionInputs(t=now, offset_px=offset, detections=dets, det_t=det_t,
                                 tof_mm=tof, tof_t=tof_t, us_fl_mm=fl, us_fl_t=fl_t,
                                 us_fr_mm=fr, us_fr_t=fr_t, estop=self.estop)
            out, st_new = decide(inp, st_in) if self.steer_cfg is None else decide(inp, st_in, self.steer_cfg)
            st_new, out = self.feed.finish(self.st, st_new, out, mode)
            self.st = st_new
            decision = out

            # a crashed thread always stops the car (the old loop stopped on any exception)
            fs_txt = ""
            scale = 1.0
            crashed = [f"{n} thread crashed ({e})" for n, e in
                       (("camera", self.cam_thread.error), ("lane", self.lane_worker.error),
                        ("detection", self.det_worker.error if self.det_worker else None)) if e]
            if crashed:
                fs_txt = crashed[0][:150]
                if self.running:
                    self.running = False
                    self.stop_reason = f"FAIL-SAFE: {fs_txt}"
                    self.message = self.stop_reason
                    self.motors.stop()
            # fail-safes (Part 4)
            if self.failsafe is not None:
                action, txt = self.failsafe.check(self, now)
                fs_txt = fs_txt or txt
                if action == "stop" and self.running:
                    self.running = False
                    self.stop_reason = f"FAIL-SAFE: {fs_txt}"
                    self.message = self.stop_reason
                    self.motors.stop()
                elif action == "slow":
                    scale = config.FAILSAFE_HOT_FACTOR
                if action in ("slow", "warn") and txt:
                    self.message = txt

            if self.running:
                state, reason = out.state, out.reason
                left, right = out.left * scale, out.right * scale
                if scale != 1.0 and left != 0:
                    reason = f"{reason} | {fs_txt}"
            else:
                state = "ESTOP" if self.estop else "STOPPED"
                reason = f"{self.stop_reason or 'press START'} | would: {out.state} {out.reason}"
                left = right = 0.0
            self.motors.drive(left, right)

            # stats
            lane_age = None if lane is None else now - lane.t_frame
            det_age = None if det is None else now - det.t_frame
            if mode == "new" and lane is not None:
                self.s_lane_lat.add(lane.t_done - lane.t_frame)
            self.s_lane_age.add(lane_age)
            self.s_det_age.add(det_age)
            if det is not None:
                self.s_det_lat.add(det.t_done - det.t_frame)
            if self._prev_tick is not None:
                self.s_period.add(now - self._prev_tick)
            self._prev_tick = now
            self.tick_meter.tick(now)
            lane_fps = self.lane_worker.meter.rate(now)
            det_fps = self.det_worker.meter.rate(now) if self.det_worker is not None else 0.0
            temp = self.monitor.temp_c if self.monitor is not None else None
            thr = self.monitor.throttled if self.monitor is not None else None

            if state != self._last_state:
                self.history.appendleft((time.strftime("%H:%M:%S"), state, reason[:120]))
                self._last_state = state

            lres = lane.payload[2] if lane is not None and len(lane.payload) > 2 else None
            self.last = dict(t=now, state=state, reason=reason, decision=decision.state,
                             decision_reason=decision.reason, left=left, right=right, offset=offset,
                             lane_offset=None if lane is None else lane.payload[0], lane_result=lres,
                             lane_mode=mode, dets=dets_raw, dets_used=dets, ignored=ignored,
                             tof=tof, us=(fl, fr), alive=alive, lane_fps=lane_fps, det_fps=det_fps,
                             ctrl_hz=self.tick_meter.rate(now), lane_age=lane_age, det_age=det_age,
                             cpu_temp=temp, throttled=thr, failsafe=fs_txt, frame_seq=self.cam_thread.seq)

            tick_ms = (time.perf_counter() - t_in) * 1000
            self.s_tick_ms.add(tick_ms)
            if self.log is not None:
                self.log.writerow([
                    f"{now - self.t_start:.3f}", self.tick_i,
                    _f(offset, ".1f" if self.steer_cfg is None else ".3f"),
                    _fmt_dets(dets_raw), _f(det_age, ".3f"),
                    _f(tof), _f(None if tof_t is None else now - tof_t, ".3f"),
                    _f(fl), _f(None if fl_t is None else now - fl_t, ".3f"),
                    _f(fr), _f(None if fr_t is None else now - fr_t, ".3f"),
                    alive, state, reason[:200], f"{left:.1f}", f"{right:.1f}", f"{self.tick_meter.rate(now):.1f}",
                    getattr(self.lane, "source", self.lane_backend), f"{lane_fps:.2f}", f"{det_fps:.2f}",
                    _f(lane_age, ".3f"), _f(None if det is None else det.t_done - det.t_frame, ".3f"),
                    _f(temp, ".1f"), "-" if thr is None else hex(thr), self.phase, int(self.running),
                    int(self.dry_run), f"{decision.state} {decision.reason}"[:120], ";".join(ignored), fs_txt,
                    f"{tick_ms:.2f}", "-" if lane is None else lane.seq])
            self.tick_i += 1
            info = self.last
        for cb in self.on_tick:
            try:
                cb(self, info)
            except Exception as e:                               # noqa: BLE001
                print(f"[runtime] on_tick callback error: {e!r}")
        if self.verbose:
            print(f"[{self.tick_i:6d}] {state:14s} L={left:+5.1f} R={right:+5.1f} off={_f(offset, '+.2f'):>6} "
                  f"lane {lane_fps:4.1f}fps det {det_fps:4.1f}fps  {reason[:80]}")
        return info

    # -- fixed-rate loop --------------------------------------------------------
    def run(self, duration=0.0, keys=None):
        """Control loop in the calling thread until stop(), duration, end of source or an exception.
        Returns 'stopped', 'duration', 'end', 'quit', 'interrupt' or 'exception'. Motors stopped on every exit."""
        t_end = None if not duration else self.clock() + duration
        next_t = self.clock()
        try:
            while not self._stop_ev.is_set():
                now = self.clock()
                if t_end is not None and now >= t_end:
                    return "duration"
                if self.cam_thread.ended and not self.cam_thread.is_alive():
                    print("End of video source.")
                    return "end"
                self.tick(now)
                if keys is not None:
                    k = keys.get()
                    if k == "q":
                        print("q pressed: stopping.")
                        return "quit"
                    if k in (" ", "e"):
                        self.cmd_estop("E-STOP key")
                        print("E-STOP latched (r to reset, g to start)")
                    elif k == "r" and self.estop:
                        with self.lock:
                            self.estop = False
                        print("E-STOP reset (still stopped; g = start)")
                    elif k == "g":
                        self.cmd_start()
                        print("START")
                    elif k == "s":
                        self.cmd_stop("STOP key")
                        print("STOP")
                next_t += self.period
                delay = next_t - self.clock()
                if delay > 0:
                    self._stop_ev.wait(delay)
                else:
                    self.overruns += 1
                    if delay < -self.period:
                        next_t = self.clock()        # fell behind by more than a tick: resync
            return "stopped"
        except KeyboardInterrupt:
            print("\nCtrl+C: stopping.")
            return "interrupt"
        except Exception as e:                                   # noqa: BLE001
            try:
                self.motors.stop()
            finally:
                print(f"\n[runtime] EXCEPTION in control loop: {e!r} -> motors stopped")
                traceback.print_exc()
                try:
                    if self.log is not None:
                        now = self.clock()
                        self.log.writerow([f"{now - (self.t_start or now):.3f}", self.tick_i, "-", "", "-", "-", "-",
                                           "-", "-", "-", "-", "-", "EXCEPTION",
                                           f"{type(e).__name__}: {e}"[:200], "0.0", "0.0", "0.0"]
                                          + ["-"] * len(EXTRA_CSV_HEADER))
                except Exception:                                # noqa: BLE001
                    pass
            return "exception"
        finally:
            try:
                self.motors.drive(0, 0)
                self.motors.stop()
            except Exception:                                    # noqa: BLE001
                pass

    def stop(self):
        self._stop_ev.set()

    def close(self):
        self.stop()
        self.stop_threads()
        for t in (self.lane_worker, self.det_worker, self.cam_thread):
            if t is not None and t.is_alive():
                t.join(timeout=2)
        self.close_log()

    def bench_summary(self):
        now = self.clock()
        mon = self.monitor
        return {
            "lane_fps": round(self.lane_worker.meter.rate(now), 2),
            "det_fps": round(self.det_worker.meter.rate(now), 2) if self.det_worker else None,
            "cam_fps": round(self.cam_thread.meter.rate(now), 2),
            "ctrl_hz": round(1.0 / self.s_period.mean(), 2) if self.s_period.n else None,
            "period_ms": self.s_period.summary(1000, 1),
            "overruns": self.overruns,
            "tick_ms": self.s_tick_ms.summary(1, 2),
            "lane_age": self.s_lane_age.summary(),
            "det_age": self.s_det_age.summary(),
            "lane_latency": self.s_lane_lat.summary(),
            "det_latency": self.s_det_lat.summary(),
            "lane_proc_ms": self.lane_worker.proc.summary(1000, 1),
            "det_proc_ms": self.det_worker.proc.summary(1000, 1) if self.det_worker else None,
            "temp_c": None if mon is None else mon.temp_c,
            "temp_max": None if mon is None else mon.temp_max,
            "throttled_seen": None if mon is None else hex(mon.throttled_seen),
            "rss_mb": None if mon is None else round(mon.rss_mb or 0, 1),
            "errors": [e for e in (self.lane_worker.error, self.det_worker and self.det_worker.error,
                                   self.cam_thread.error) if e],
        }


def _fmt_dets(dets):
    return ";".join(f"{d['cls']}:{d['conf']:.2f}:{d.get('box_height_ratio', 0):.2f}:{d.get('cx_ratio', 0):.2f}"
                    for d in dets)
