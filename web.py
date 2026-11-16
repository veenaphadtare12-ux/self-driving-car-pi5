"""
Demo control page on port 8001 (UFLD Part 5), stdlib only (http.server). Design from ~/car_project/car/web.py.

    python main_demo.py --web [--dry-run] [--source FOLDER_OR_VIDEO] [--lane ufld]

The page only sends commands and shows state; every safety rule stays in decision.py / runtime.py /
failsafe.py. The server runs in daemon threads: if it crashes or a browser disconnects, the control
loop keeps running.

  * starts STOPPED with DRY RUN on; EMERGENCY STOP stops at once (web thread) and turns DRY RUN back on;
    E-STOP works from ANY browser (a stop is always safe); everything else needs control
  * one controlling browser; others are view-only. Control can be taken while the car is stopped, or when
    the controller's heartbeat has been missing for CONTROL_TAKEOVER_S. While a page controls, a missing
    heartbeat for FAILSAFE_HEARTBEAT_S (1.5 s) while driving stops the car (failsafe.py)
  * phases P1-P7 (changing phase stops the car), lane backend (Hough / UFLD v2_same / v2_plus / v2_same_int8, only stopped),
    RECORD (annotated view -> videos/<phase>_<time>.mp4 at the stream rate + logs/<phase>_<time>.csv)
  * live view: MJPEG ~10 fps, 480x360, overlays drawn in a separate thread, only while someone watches or records
  * settings sliders (base speed capped at WEB_BASE_SPEED_MAX, Kp/Kd of the active backend, trims): applied live,
    "Save" writes web_settings.json (loaded only by --web; config.py is never edited)
  * status: last 10 state changes, indicators (camera, ToF, ultrasonics, IMU), rehearsal tracker (rehearsal_log.csv)
"""
import csv
import json
import os
import secrets
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import config
from runtime import PHASES

HERE = os.path.dirname(os.path.abspath(__file__))
SETTINGS_PATH = os.path.join(HERE, "web_settings.json")
REHEARSAL_PATH = os.path.join(HERE, "rehearsal_log.csv")
VIDEO_DIR = os.path.join(HERE, "videos")
STREAM_FPS = 10
STREAM_SIZE = (480, 360)
CONTROL_TAKEOVER_S = 5.0
LANE_CHOICES = {"hough": ("hough", None), "ufld:v2_same": ("ufld", "v2_same"), "ufld:v2_plus": ("ufld", "v2_plus"),
                "ufld:v2_same_int8": ("ufld", "v2_same_int8")}
PARAMS = {  # page name: (config name for hough, config name for ufld, min, max, step)
    "base_speed": ("BASE_SPEED", "BASE_SPEED", 0, None, 1),
    "kp": ("STEER_KP", "STEER_KP_UFLD", 0, None, None),
    "kd": ("STEER_KD", "STEER_KD_UFLD", 0, None, None),
    "left_trim": ("LEFT_TRIM", "LEFT_TRIM", 0.5, 1.0, 0.01),
    "right_trim": ("RIGHT_TRIM", "RIGHT_TRIM", 0.5, 1.0, 0.01),
}
STATE_COLORS = {"DRIVE": (0, 200, 0), "CAUTION": (0, 255, 255), "STOP_OBSTACLE": (0, 0, 255),
                "STOP_RED": (0, 0, 255), "STOP_SIGN": (0, 0, 200), "STOP_LANE_LOST": (0, 165, 255),
                "ESTOP": (255, 0, 255), "STOPPED": (160, 160, 160)}


def base_speed_max():
    return float(getattr(config, "WEB_BASE_SPEED_MAX", config.MAX_SPEED))


def load_settings(path=None):
    """Apply saved slider values (web mode only)."""
    path = path or SETTINGS_PATH
    try:
        with open(path) as f:
            vals = json.load(f)
    except (OSError, ValueError):
        return {}
    applied = {}
    for k, v in vals.items():
        if k in {n for p in PARAMS.values() for n in p[:2]} and isinstance(v, (int, float)):
            if k == "BASE_SPEED":
                v = min(float(v), base_speed_max())
            setattr(config, k, v)
            applied[k] = v
    return applied


# ---------------------------------------------------------------------------
# rendering
# ---------------------------------------------------------------------------
def _fmt(v, spec=".0f"):
    return "-" if v is None else format(v, spec)


def render(frame, info, rt, rec=False):
    from lanes.ufld import draw_lane_result
    from obstacle_yolo import draw_detections, draw_regions
    if frame is None:
        img = np.zeros((config.CAM_HEIGHT, config.CAM_WIDTH, 3), np.uint8)
        cv2.putText(img, "CAMERA NOT CONNECTED", (70, 250), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 255), 3)
    else:
        img = frame.copy()
        lres = info.get("lane_result")
        if lres is not None:
            if lres.source == "hough":
                for pts, col in ((lres.left, (0, 255, 0)), (lres.right, (0, 255, 0))):
                    if len(pts) >= 2:
                        p = np.array([(int(x), int(y)) for y, x in sorted(pts.items())], np.int32)
                        cv2.polylines(img, [p], False, col, 3)
                img = draw_lane_result(img, type(lres)(**{**lres.__dict__, "left": {}, "right": {}}))
            else:
                img = draw_lane_result(img, lres)
        img = draw_regions(draw_detections(img, info.get("dets") or []))
    h, w = img.shape[:2]
    state = info.get("state", "-")
    cv2.rectangle(img, (0, 0), (w, 64), (0, 0, 0), -1)
    cv2.putText(img, f"{rt.phase} {state}  {info.get('reason', '')[:60]}", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                STATE_COLORS.get(state, (255, 255, 255)), 2)
    us = info.get("us") or (None, None)
    cv2.putText(img, f"L {info.get('left', 0):+5.1f} R {info.get('right', 0):+5.1f} | ToF {_fmt(info.get('tof'))} "
                f"US {_fmt(us[0])}/{_fmt(us[1])} | lane {info.get('lane_fps', 0):.1f} det {info.get('det_fps', 0):.1f} "
                f"ctrl {info.get('ctrl_hz', 0):.0f}Hz", (8, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)
    ig = info.get("ignored") or []
    cv2.putText(img, f"CPU {_fmt(info.get('cpu_temp'), '.1f')}C | {'DRY RUN' if rt.dry_run else 'MOTORS LIVE'}"
                + (f" | ignored by phase: {','.join(ig)}" if ig else ""), (8, 59),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 255) if rt.dry_run else (0, 0, 255), 1)
    if rec:
        cv2.circle(img, (w - 20, 20), 9, (0, 0, 255), -1)
    return img


class View(threading.Thread):
    """Draws the annotated view at STREAM_FPS while someone watches or a recording runs."""

    def __init__(self, rt, fps=STREAM_FPS, size=STREAM_SIZE):
        super().__init__(name="view", daemon=True)
        self.rt, self.period, self.size = rt, 1.0 / fps, size
        self.cond = threading.Condition()
        self.jpeg = None
        self.seq = 0
        self.viewers = 0
        self.writer = None
        self.rec_path = None
        self.rec_frames = 0
        self.lock = threading.Lock()
        self.fps_meter = None
        self._stop_ev = threading.Event()
        self.error = None

    def run(self):
        from runtime import RateMeter
        self.fps_meter = RateMeter()
        while not self._stop_ev.is_set():
            t0 = time.monotonic()
            try:
                with self.lock:
                    writer = self.writer
                if self.viewers > 0 or writer is not None:
                    _, fr = self.rt.cam_thread.slot.get()
                    frame = None if fr is None or time.monotonic() - fr.t > 2.0 else fr.img
                    img = render(frame, dict(self.rt.last), self.rt, rec=writer is not None)
                    img = cv2.resize(img, self.size, interpolation=cv2.INTER_AREA)
                    if writer is not None:
                        with self.lock:
                            if self.writer is not None:
                                self.writer.write(img)
                                self.rec_frames += 1
                    if self.viewers > 0:
                        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                        if ok:
                            with self.cond:
                                self.jpeg, self.seq = buf.tobytes(), self.seq + 1
                                self.cond.notify_all()
                    self.fps_meter.tick()
            except Exception as e:                               # noqa: BLE001
                self.error = repr(e)
                traceback.print_exc()
            self._stop_ev.wait(max(0.0, self.period - (time.monotonic() - t0)))

    def start_recording(self, phase):
        os.makedirs(VIDEO_DIR, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(VIDEO_DIR, f"{phase}_{stamp}.mp4")
        w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), 1.0 / self.period, self.size)
        if not w.isOpened():
            raise RuntimeError(f"cannot open video writer {path}")
        with self.lock:
            self.writer, self.rec_path, self.rec_frames = w, path, 0
        return path, stamp

    def stop_recording(self):
        with self.lock:
            w, path, self.writer = self.writer, self.rec_path, None
        if w is not None:
            w.release()
        return path

    def stop(self):
        self._stop_ev.set()
        self.stop_recording()


# ---------------------------------------------------------------------------
# shared web state
# ---------------------------------------------------------------------------
class WebState:
    def __init__(self, rt, real_motors_factory=None, settings_path=None, rehearsal_path=None):
        self.rt = rt
        self.real_motors_factory = real_motors_factory
        self.settings_path = settings_path or SETTINGS_PATH
        self.rehearsal_path = rehearsal_path or REHEARSAL_PATH
        self.view = View(rt)
        self.lock = threading.Lock()
        self.token = None
        self.token_seen = None
        self.switching = None
        self.rec_csv = None
        self.rehearsal = self._load_rehearsal()
        self.cmd_ms = []
        if rt.failsafe is None:
            from failsafe import FailSafe
            rt.failsafe = FailSafe()

    # -- control ownership --------------------------------------------------------
    def controller_alive(self, now=None):
        now = time.monotonic() if now is None else now
        return self.token is not None and self.token_seen is not None and now - self.token_seen < CONTROL_TAKEOVER_S

    def claim(self, token=None, force=False):
        with self.lock:
            now = time.monotonic()
            if token is not None and token == self.token:
                self.token_seen = now
                return token
            if self.token is None or not self.controller_alive(now) or (force and not self.rt.running):
                self.token, self.token_seen = secrets.token_hex(8), now
                self.rt.failsafe.heartbeat_required = True
                self.rt.failsafe.heartbeat(now)
                return self.token
            return None

    def is_controller(self, token):
        return token is not None and token == self.token

    def heartbeat(self, token):
        if not self.is_controller(token):
            return False
        now = time.monotonic()
        self.token_seen = now
        self.rt.failsafe.heartbeat(now)
        return True

    # -- settings -------------------------------------------------------------------
    def params(self):
        ufld = self.rt.lane_backend == "ufld"
        return {k: getattr(config, v[1] if ufld else v[0]) for k, v in PARAMS.items()}

    def set_params(self, vals):
        from lanes import steer_cfg
        ufld = self.rt.lane_backend == "ufld"
        out = {}
        for k, v in vals.items():
            if k not in PARAMS:
                continue
            name = PARAMS[k][1] if ufld else PARAMS[k][0]
            v = float(v)
            lo, hi = PARAMS[k][2], PARAMS[k][3] if k != "base_speed" else base_speed_max()
            if hi is None:
                hi = 1e9
            if not lo <= v <= hi:
                raise ValueError(f"{k} must be {lo:g}..{hi:g}")
            setattr(config, name, v)
            out[name] = v
        with self.rt.lock:
            self.rt.steer_cfg = steer_cfg(self.rt.lane_backend)
        return out

    def save_params(self):
        names = sorted({n for p in PARAMS.values() for n in p[:2]})
        vals = {n: getattr(config, n) for n in names}
        with open(self.settings_path, "w") as f:
            json.dump(vals, f, indent=1)
        return vals

    # -- rehearsal ------------------------------------------------------------------
    def _load_rehearsal(self):
        res = {p: [] for p in PHASES}
        try:
            with open(self.rehearsal_path) as f:
                for r in csv.DictReader(f):
                    if r.get("phase") in res:
                        res[r["phase"]].append(r.get("result"))
        except OSError:
            pass
        return res

    def add_rehearsal(self, phase, result, note=""):
        if phase not in PHASES or result not in ("PASS", "FAIL"):
            raise ValueError("phase P1..P7 and result PASS/FAIL")
        new = not os.path.exists(self.rehearsal_path)
        with open(self.rehearsal_path, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["time", "phase", "result", "lane_source", "video", "note"])
            w.writerow([time.strftime("%Y-%m-%d %H:%M:%S"), phase, result, getattr(self.rt.lane, "source", ""),
                        os.path.basename(self.view.rec_path or ""), note])
        self.rehearsal[phase].append(result)

    def rehearsal_summary(self):
        out = {}
        for p, rs in self.rehearsal.items():
            streak = 0
            for r in reversed(rs):
                if r != "PASS":
                    break
                streak += 1
            out[p] = {"pass": rs.count("PASS"), "fail": rs.count("FAIL"), "streak": streak}
        return out

    # -- recording ------------------------------------------------------------------
    def record(self, on):
        rt = self.rt
        if on and self.view.writer is None:
            path, stamp = self.view.start_recording(rt.phase)
            self.rec_csv = os.path.join(config.LOG_DIR, f"{rt.phase}_{stamp}.csv")
            with rt.lock:
                self._prev_log = rt.log_path
                rt.open_log(self.rec_csv)
            return path
        if not on and self.view.writer is not None:
            path = self.view.stop_recording()
            with rt.lock:
                rt.open_log(os.path.join(config.LOG_DIR, time.strftime("run_%Y%m%d_%H%M%S.csv")))
            self.rec_csv = None
            return path
        return None

    # -- lane switch (model load runs in the background) ----------------------------
    def switch_lane(self, choice):
        if choice not in LANE_CHOICES:
            raise ValueError(f"lane must be one of {list(LANE_CHOICES)}")
        if self.rt.running:
            raise RuntimeError("stop the car before changing the lane backend")
        if self.switching:
            raise RuntimeError(f"already switching to {self.switching}")
        backend, model = LANE_CHOICES[choice]
        self.switching = choice

        def work():
            try:
                self.rt.set_lane(backend, model)
                self.rt.message = f"lane backend: {self.rt.lane.source}"
            except Exception as e:                               # noqa: BLE001
                self.rt.message = f"lane switch failed: {e}"
            finally:
                self.switching = None
        threading.Thread(target=work, name="lane-switch", daemon=True).start()

    # -- status -----------------------------------------------------------------------
    def status(self, token=None):
        rt = self.rt
        info = dict(rt.last)
        now = time.monotonic()
        cam_age = None if rt.cam_thread.last_t is None else now - rt.cam_thread.last_t
        ind = {"camera": cam_age is not None and cam_age < 1.0}
        if rt.sensors is not None:
            try:
                tv, tt = rt.sensors.tof_reading()
                (fv, ft), (rv, rt_) = rt.sensors.ultrasonic_readings()
                fresh = lambda t: t is not None and now - t < 1.0
                ind.update(tof=fresh(tt), us_left=fresh(ft), us_right=fresh(rt_),
                           imu=getattr(rt.sensors, "gyro", None) is not None)
            except Exception:                                    # noqa: BLE001
                ind.update(tof=False, us_left=False, us_right=False, imu=False)
        else:
            ind.update(tof=None, us_left=None, us_right=None, imu=None)
        lres = info.get("lane_result")
        fs = rt.failsafe
        r = lambda v, n=2: None if v is None else round(v, n)
        return {
            "state": info.get("state", "STOPPED"), "reason": info.get("reason", ""),
            "decision": info.get("decision"), "running": rt.running, "estop": rt.estop, "dry_run": rt.dry_run,
            "dry_locked": self.real_motors_factory is None, "phase": rt.phase,
            "phases": {k: v[0] for k, v in PHASES.items()},
            "lane": getattr(rt.lane, "source", rt.lane_backend), "lane_choices": list(LANE_CHOICES),
            "switching": self.switching, "left": r(info.get("left"), 1), "right": r(info.get("right"), 1),
            "offset": r(info.get("lane_offset"), 3), "lines": None if lres is None else lres.lines,
            "confidence": None if lres is None else r(lres.confidence),
            "lane_fps": r(info.get("lane_fps"), 1), "det_fps": r(info.get("det_fps"), 1),
            "ctrl_hz": r(info.get("ctrl_hz"), 1), "lane_age": r(info.get("lane_age")), "det_age": r(info.get("det_age")),
            "cpu_temp": r(info.get("cpu_temp"), 1),
            "throttled": None if info.get("throttled") is None else hex(info["throttled"]),
            "undervolt": bool((info.get("throttled") or 0) & 0x1) or bool(fs and fs.undervolt_seen),
            "tof": r(info.get("tof"), 0), "us": [r(v, 0) for v in (info.get("us") or (None, None))],
            "ignored": info.get("ignored") or [], "failsafe": info.get("failsafe", ""),
            "offer_hough": bool(fs and fs.offer_hough), "message": rt.message,
            "history": list(rt.history), "indicators": ind,
            "recording": self.view.writer is not None, "rec_path": self.view.rec_path and os.path.basename(self.view.rec_path),
            "rec_csv": self.rec_csv and os.path.basename(self.rec_csv), "view_fps": r(self.view.fps_meter.rate() if self.view.fps_meter else 0, 1),
            "params": self.params(), "base_speed_max": base_speed_max(),
            "rehearsal": self.rehearsal_summary(),
            "controller": self.is_controller(token), "controlled": self.controller_alive(now),
        }


# ---------------------------------------------------------------------------
# page
# ---------------------------------------------------------------------------
PAGE = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>Car Demo</title>
<style>
:root{--bg:#111;--fg:#eee;--mut:#9a9a9a;--card:#1d1d1d;--go:#1f9d55;--stop:#c0392b;--warn:#e0a020;--sel:#2d5d8a}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:16px system-ui,sans-serif}
main{max-width:760px;margin:0 auto;padding:0 16px 40px}
.top{position:sticky;top:0;background:var(--bg);padding:8px 0 6px;z-index:5}
h1{font-size:17px;margin:2px 0 6px;display:flex;gap:8px;align-items:center;flex-wrap:wrap}
#state{font-weight:700;padding:2px 8px;border-radius:6px}
img{width:100%;display:block;border-radius:8px;background:#000;aspect-ratio:4/3}
.row{display:flex;gap:8px;margin-top:8px}.row>*{flex:1}
button{width:100%;font-size:18px;font-weight:700;padding:14px 6px;border:0;border-radius:10px;color:#fff;background:#444}
button:disabled{opacity:.35}
#start{background:var(--go)}#stopb{background:#555}#estop{background:var(--stop);font-size:26px;padding:20px}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-top:8px}
.grid div{background:var(--card);border-radius:8px;padding:6px 8px}.grid b{display:block;font-size:19px;font-variant-numeric:tabular-nums}
.grid span{font-size:12px;color:var(--mut)}
.card{background:var(--card);border-radius:10px;padding:10px 12px;margin-top:10px}
.card h2{font-size:14px;color:var(--mut);margin:0 0 6px;font-weight:600}
.phases{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}.phases button{font-size:16px;padding:10px 4px}
.phases button.on{background:var(--sel);outline:3px solid #8fc3ff}
#phasedesc{font-size:13px;color:var(--mut);margin-top:6px}
label{display:flex;justify-content:space-between;font-size:14px;color:var(--mut);margin-top:6px}
label b{color:var(--fg)}input[type=range]{width:100%}
select{font-size:16px;padding:6px;border-radius:6px;background:#222;color:var(--fg);border:1px solid #444}
#msg{min-height:1.3em;color:var(--warn);margin-top:4px;font-size:14px}
.sw{display:flex;align-items:center;justify-content:space-between;gap:8px;margin-top:8px}.sw input[type=checkbox]{width:28px;height:28px}
.ind span{display:inline-block;margin:2px 6px 2px 0;padding:2px 8px;border-radius:12px;font-size:13px;background:#333}
.ind .ok{background:#1f5f3a}.ind .bad{background:#7a2620}
table{width:100%;border-collapse:collapse;font-size:13px}td,th{padding:3px 4px;text-align:left;border-bottom:1px solid #2a2a2a}
.small{font-size:12px;color:var(--mut)}#ctl{font-size:13px}
.rh button{font-size:14px;padding:6px;width:auto}
</style></head><body><main>
<div class="top">
<h1>Car Demo <span id="state">-</span> <span id="dry" style="color:var(--warn)"></span> <span id="rec" style="color:#f55"></span></h1>
<button id="estop" onclick="cmd('estop')">EMERGENCY STOP</button>
<div class="row"><button id="start" onclick="cmd('start')">START</button><button id="stopb" onclick="cmd('stop')">STOP</button></div>
<div id="msg"></div>
</div>
<img id="view" src="/stream" alt="live camera with lane and detection overlays">
<div class="small" id="reason"></div>
<div class="grid">
<div><span>lane fps</span><b id="lane_fps">-</b></div><div><span>detect fps</span><b id="det_fps">-</b></div><div><span>control Hz</span><b id="ctrl_hz">-</b></div>
<div><span>lane age s</span><b id="lane_age">-</b></div><div><span>detect age s</span><b id="det_age">-</b></div><div><span>CPU &deg;C</span><b id="cpu_temp">-</b></div>
<div><span>offset</span><b id="offset">-</b></div><div><span>left / right %</span><b id="lr">-</b></div><div><span>ToF / US mm</span><b id="dist">-</b></div>
</div>
<div class="card"><h2>PHASE (changing phase stops the car)</h2><div class="phases" id="phases"></div><div id="phasedesc"></div></div>
<div class="card"><h2>RUN</h2>
<div class="sw"><span>DRY RUN (motors never move)</span><input type="checkbox" id="dryrun"></div>
<div class="sw"><span>Lane</span><select id="lane"></select><button id="hough" style="display:none;width:auto;font-size:14px;padding:8px;background:var(--warn)" onclick="post('/lane',{choice:'hough'})">Switch to Hough</button></div>
<div class="sw"><span>RECORD video + CSV</span><input type="checkbox" id="record"></div>
<div class="small" id="recinfo"></div>
<div class="sw"><span id="ctl">-</span><button id="take" style="width:auto;font-size:14px;padding:8px;background:var(--sel)" onclick="claim(true)">Take control</button></div>
</div>
<div class="card"><h2>STATUS</h2><div class="ind" id="ind"></div><div class="small" id="thr"></div>
<table id="hist"></table></div>
<div class="card"><h2>REHEARSAL (goal: 5 PASS in a row)</h2><table id="rh"></table></div>
<div class="card"><h2>SETTINGS (applied live; Save keeps them for the next --web start)</h2>
<label>Base speed % (max <span id="bmax"></span>) <b id="v_base_speed"></b></label><input type="range" id="base_speed" min="0" step="1">
<label>Kp <b id="v_kp"></b></label><input type="range" id="kp" min="0" step="0.01">
<label>Kd <b id="v_kd"></b></label><input type="range" id="kd" min="0" step="0.01">
<label>Left trim <b id="v_left_trim"></b></label><input type="range" id="left_trim" min="0.5" max="1" step="0.01">
<label>Right trim <b id="v_right_trim"></b></label><input type="range" id="right_trim" min="0.5" max="1" step="0.01">
<div class="row"><button style="background:var(--sel);font-size:16px;padding:10px" onclick="post('/save_params',{})">Save</button></div>
</div>
</main><script>
const $=id=>document.getElementById(id);let token=sessionStorage.getItem("tok");let S={};let dragging={};
const P=["base_speed","kp","kd","left_trim","right_trim"];
async function post(u,b){b=b||{};b.token=token;try{const r=await fetch(u,{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(b)});
const j=await r.json();if(j.error)$("msg").textContent=j.error;refresh();return j}catch(e){$("msg").textContent="no connection to the car"}}
function cmd(a){post("/cmd",{action:a})}
async function claim(force){const j=await post("/claim",{force:!!force});if(j&&j.token){token=j.token;sessionStorage.setItem("tok",token)}}
setInterval(()=>{if(token)fetch("/heartbeat",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({token})}).then(r=>r.json()).then(j=>{if(!j.ok){token=null;sessionStorage.removeItem("tok")}}).catch(()=>{})},500);
P.forEach(k=>{const e=$(k);e.oninput=()=>{dragging[k]=1;$("v_"+k).textContent=e.value};e.onchange=async()=>{await post("/params",{[k]:parseFloat(e.value)});dragging[k]=0}});
$("dryrun").onchange=async e=>{const want=e.target.checked;if(!want&&!confirm("Switch DRY RUN off? The motors WILL move when you press START."))return refresh();await post("/dryrun",{on:want})};
$("record").onchange=e=>post("/record",{on:e.target.checked});
$("lane").onchange=e=>post("/lane",{choice:e.target.value});
const col={DRIVE:"#1f9d55",CAUTION:"#e0a020",STOPPED:"#555",ESTOP:"#c0392b",STOP_OBSTACLE:"#c0392b",STOP_RED:"#c0392b",STOP_SIGN:"#a03020",STOP_LANE_LOST:"#d07000"};
const f=(v,d)=>v===null||v===undefined?"-":Number(v).toFixed(d);
function phases(s){const el=$("phases");if(!el.childElementCount){for(const p in s.phases){const b=document.createElement("button");b.textContent=p;b.id="ph_"+p;b.onclick=()=>post("/phase",{phase:p});el.appendChild(b)}}
for(const p in s.phases){const b=$("ph_"+p);b.className=p==s.phase?"on":"";b.disabled=!s.controller}$("phasedesc").textContent=s.phase+": "+s.phases[s.phase]}
async function refresh(){try{const s=await(await fetch("/status?token="+(token||""))).json();S=s;
$("state").textContent=s.state;$("state").style.background=col[s.state]||"#333";
$("dry").textContent=s.dry_run?"DRY RUN":"MOTORS LIVE";$("dryrun").checked=s.dry_run;$("dryrun").disabled=!s.controller||s.dry_locked;
$("rec").textContent=s.recording?"● REC":"";$("record").checked=s.recording;$("record").disabled=!s.controller;
$("recinfo").textContent=s.recording?("recording "+s.rec_path+" + "+s.rec_csv):"";
$("reason").textContent=s.reason;
["lane_fps","det_fps","ctrl_hz"].forEach(k=>$(k).textContent=f(s[k],1));["lane_age","det_age"].forEach(k=>$(k).textContent=f(s[k],2));
$("cpu_temp").textContent=f(s.cpu_temp,1);$("offset").textContent=f(s.offset,2);$("lr").textContent=f(s.left,0)+" / "+f(s.right,0);
$("dist").textContent=f(s.tof,0)+" / "+f(s.us[0],0)+","+f(s.us[1],0);
$("msg").textContent=s.message||"";phases(s);
const ls=$("lane");if(!ls.childElementCount)s.lane_choices.forEach(c=>{const o=document.createElement("option");o.value=o.textContent=c;ls.appendChild(o)});
ls.value=s.lane;ls.disabled=!s.controller||s.running||!!s.switching;$("hough").style.display=s.offer_hough&&!s.running&&s.controller?"":"none";
["start","stopb"].forEach(k=>$(k).disabled=!s.controller);
$("ctl").textContent=s.controller?"You control the car":(s.controlled?"VIEW ONLY (another browser controls)":"No browser controls the car");
$("take").style.display=s.controller?"none":"";
const ind=s.indicators;$("ind").innerHTML=Object.keys(ind).map(k=>`<span class="${ind[k]===null?'':(ind[k]?'ok':'bad')}">${k} ${ind[k]===null?'off':(ind[k]?'OK':'NO')}</span>`).join("");
$("thr").textContent="throttled "+(s.throttled||"-")+(s.undervolt?"  UNDER-VOLTAGE!":"")+(s.switching?"  switching lane to "+s.switching+"...":"")+"  view "+f(s.view_fps,1)+" fps";
$("hist").innerHTML="<tr><th>time</th><th>state</th><th>reason</th></tr>"+s.history.map(h=>`<tr><td>${h[0]}</td><td>${h[1]}</td><td>${h[2]}</td></tr>`).join("");
$("rh").innerHTML="<tr><th>phase</th><th>pass</th><th>fail</th><th>in a row</th><th></th></tr>"+Object.keys(s.rehearsal).map(p=>{const r=s.rehearsal[p];
return `<tr><td>${p}</td><td>${r.pass}</td><td>${r.fail}</td><td><b>${r.streak}</b>/5</td><td class="rh"><button ${s.controller?"":"disabled"} style="background:#1f6f45" onclick="post('/rehearsal',{phase:'${p}',result:'PASS'})">PASS</button> <button ${s.controller?"":"disabled"} style="background:#7a2620" onclick="post('/rehearsal',{phase:'${p}',result:'FAIL'})">FAIL</button></td></tr>`}).join("");
$("bmax").textContent=s.base_speed_max;$("base_speed").max=s.base_speed_max;$("kp").max=Math.max(1,s.params.kp*3);$("kd").max=Math.max(0.5,s.params.kd*3);
P.forEach(k=>{$(k).disabled=!s.controller;if(!dragging[k]){$(k).value=s.params[k];$("v_"+k).textContent=s.params[k]}});
}catch(e){$("msg").textContent="no connection to the car"}}
claim(false);setInterval(refresh,500);refresh();
</script></body></html>"""


def make_handler(ws):
    page = PAGE.encode()

    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj).encode(), "application/json")

        def do_GET(self):
            try:
                path, _, q = self.path.partition("?")
                if path == "/":
                    return self._send(200, page, "text/html; charset=utf-8")
                if path == "/status":
                    tok = dict(p.split("=", 1) for p in q.split("&") if "=" in p).get("token") or None
                    return self._json(ws.status(tok))
                if path == "/stream":
                    return self._stream()
                return self._send(404, b"not found", "text/plain")
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as e:                               # noqa: BLE001
                traceback.print_exc()
                try:
                    self._json({"error": repr(e)}, 500)
                except Exception:                                # noqa: BLE001
                    pass

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            v = ws.view
            with v.cond:
                v.viewers += 1
            try:
                seq = -1
                while True:
                    with v.cond:
                        v.cond.wait_for(lambda: v.seq != seq, timeout=2.0)
                        jpg, seq = v.jpeg, v.seq
                    if jpg is None:
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                     + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with v.cond:
                    v.viewers -= 1
                self.close_connection = True

        def do_POST(self):
            t0 = time.perf_counter()
            try:
                n = int(self.headers.get("Content-Length") or 0)
                try:
                    body = json.loads(self.rfile.read(n) or b"{}")
                except ValueError:
                    body = {}
                res = self._post(self.path, body)
            except Exception as e:                               # noqa: BLE001
                traceback.print_exc()
                res = ({"error": f"{type(e).__name__}: {e}"}, 500)
            obj, code = res if isinstance(res, tuple) else (res, 200)
            ws.cmd_ms.append((self.path, (time.perf_counter() - t0) * 1000))
            ws.cmd_ms = ws.cmd_ms[-200:]
            try:
                self._json(obj, code)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def _post(self, path, b):
            rt, tok = ws.rt, b.get("token")
            if path == "/heartbeat":
                return {"ok": ws.heartbeat(tok)}
            if path == "/claim":
                t = ws.claim(tok, force=bool(b.get("force")))
                if t is None:
                    return {"error": "another browser controls the car (view only)" + (
                        "; stop the car first to take control" if rt.running else "")}, 409
                return {"ok": True, "token": t}
            if path == "/cmd" and b.get("action") == "estop":          # E-STOP from any browser
                rt.cmd_estop("EMERGENCY STOP button")
                return {"ok": True}
            if not ws.is_controller(tok):
                return {"error": "view only: another browser controls the car (or press Take control)"}, 403
            ws.heartbeat(tok)
            if path == "/cmd":
                a = b.get("action")
                if a == "start":
                    if ws.switching:
                        return {"error": "wait: lane backend is loading"}, 409
                    rt.cmd_start()
                elif a == "stop":
                    rt.cmd_stop("STOP button")
                else:
                    return {"error": f"unknown action {a!r}"}, 400
                return {"ok": True}
            if path == "/phase":
                rt.set_phase(b.get("phase"))
                return {"ok": True, "phase": rt.phase}
            if path == "/dryrun":
                on = bool(b.get("on"))
                try:
                    rt.set_dry_run(on, ws.real_motors_factory)
                except RuntimeError as e:
                    return {"error": str(e)}, 409
                return {"ok": True, "dry_run": rt.dry_run}
            if path == "/lane":
                try:
                    ws.switch_lane(b.get("choice"))
                except (RuntimeError, ValueError) as e:
                    return {"error": str(e)}, 409
                return {"ok": True}
            if path == "/record":
                p = ws.record(bool(b.get("on")))
                return {"ok": True, "path": p and os.path.basename(p)}
            if path == "/params":
                try:
                    return {"ok": True, "set": ws.set_params({k: v for k, v in b.items() if k != "token"})}
                except ValueError as e:
                    return {"error": str(e)}, 400
            if path == "/save_params":
                return {"ok": True, "saved": ws.save_params()}
            if path == "/rehearsal":
                try:
                    ws.add_rehearsal(b.get("phase"), b.get("result"), b.get("note", ""))
                except ValueError as e:
                    return {"error": str(e)}, 400
                return {"ok": True}
            return {"error": "not found"}, 404

    return H


def start(rt, port=8001, real_motors_factory=None, host="0.0.0.0"):
    """Starts the page in daemon threads; returns the server (srv.ws = WebState)."""
    applied = load_settings()
    if applied:
        from lanes import steer_cfg
        rt.steer_cfg = steer_cfg(rt.lane_backend)
        print(f"[web] settings from {os.path.basename(SETTINGS_PATH)}: {applied}")
    ws = WebState(rt, real_motors_factory)
    rt.running = False
    rt.set_dry_run(True)
    ws.view.start()
    srv = ThreadingHTTPServer((host, port), make_handler(ws))
    srv.daemon_threads = True
    srv.ws = ws
    threading.Thread(target=srv.serve_forever, name="web", daemon=True).start()
    orig_shutdown = srv.shutdown

    def shutdown():
        ws.view.stop()
        orig_shutdown()
    srv.shutdown = shutdown
    return srv
