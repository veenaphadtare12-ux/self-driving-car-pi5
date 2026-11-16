#!/usr/bin/env python3
"""Soak test driver (UFLD Part 6) for a RUNNING demo page: acts as the controlling browser (heartbeat), keeps the
MJPEG stream open, RECORD on, START (DRY RUN only), samples /status + the server's memory every 5 s.
Any fail-safe stop is recorded and START is pressed again.

  python soak_check.py --pid SERVER_PID --minutes 15 --out logs/soak_replay

Summary: fps (control / lane / detection / camera-in-view) mean-min-max, ages at the control tick (from the run CSV,
every tick), temperature curve (per minute), RSS at 1 min vs end (growth %), exceptions in the server log,
recorded video frames vs expected, fail-safe stops.
"""
import argparse
import csv
import json
import os
import statistics
import subprocess
import threading
import time

import cv2

from check_web_e2e import Browser, Stream


def rss_mb(pid):
    with open(f"/proc/{pid}/status") as f:
        for line in f:
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) / 1024.0
    return None


def _vc(*args):
    try:
        return subprocess.run(["vcgencmd", *args], capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception as e:                                       # noqa: BLE001
        return f"err {e}"


class PowerLog(threading.Thread):
    """get_throttled + EXT5V supply voltage + SoC temperature every `period` s, one flushed line each
    (so the file survives a power cut)."""

    def __init__(self, path, period=10.0):
        super().__init__(daemon=True)
        self.path, self.period, self.ev = path, period, threading.Event()
        self.rows = []

    def run(self):
        t0 = time.monotonic()
        with open(self.path, "w", buffering=1) as f:
            f.write("wall_time,t,throttled,ext5v_v,temp_c\n")
            while True:
                th = _vc("get_throttled").split("=")[-1]
                v = _vc("pmic_read_adc", "EXT5V_V").split("=")[-1].rstrip("V")
                tc = _vc("measure_temp").split("=")[-1].rstrip("'C")
                row = [time.strftime("%H:%M:%S"), f"{time.monotonic() - t0:.0f}", th, v, tc]
                self.rows.append(row)
                f.write(",".join(row) + "\n")
                os.fsync(f.fileno())
                if self.ev.wait(self.period):
                    return

    def stop(self):
        self.ev.set()
        self.join(timeout=10)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://127.0.0.1:8001")
    ap.add_argument("--pid", type=int, required=True)
    ap.add_argument("--minutes", type=float, default=15)
    ap.add_argument("--server-log", default=None)
    ap.add_argument("--out", default="logs/soak")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    plog = PowerLog(os.path.join(a.out, "power.csv"))
    plog.start()
    live = open(os.path.join(a.out, "samples_live.jsonl"), "w", buffering=1)
    A = Browser(a.base)
    s = A.status()
    assert s["dry_run"], "motors are LIVE - soak only runs in DRY RUN"
    assert A.claim() == 200, "cannot take control"
    A.start_heartbeat()
    st = Stream(a.base)
    A.post("/phase", {"phase": "P7"})
    code, obj, _ = A.post("/record", {"on": True})
    t_rec = time.monotonic()
    A.post("/cmd", {"action": "start"})
    t0 = time.monotonic()
    samples, stops = [], []
    rss_1min = None
    dur = a.minutes * 60
    last_print = 0
    while time.monotonic() - t0 < dur:
        time.sleep(5)
        el = time.monotonic() - t0
        s = A.status()
        r = rss_mb(a.pid)
        if rss_1min is None and el >= 60:
            rss_1min = r
        samples.append({"t": round(el, 1), "ctrl_hz": s["ctrl_hz"], "lane_fps": s["lane_fps"], "det_fps": s["det_fps"],
                        "view_fps": s["view_fps"], "lane_age": s["lane_age"], "det_age": s["det_age"],
                        "cpu_temp": s["cpu_temp"], "throttled": s["throttled"], "state": s["state"],
                        "running": s["running"], "rss_mb": round(r, 1), "stream_frames": st.frames})
        live.write(json.dumps(samples[-1]) + "\n")
        if not s["running"]:
            stops.append({"t": round(el, 1), "reason": s["message"] or s["reason"]})
            print(f"[{el:6.0f}s] STOPPED: {stops[-1]['reason']} -> START again", flush=True)
            A.post("/cmd", {"action": "start"})
        if el - last_print >= 60:
            last_print = el
            print(f"[{el:6.0f}s] ctrl {s['ctrl_hz']} lane {s['lane_fps']} det {s['det_fps']} view {s['view_fps']} "
                  f"temp {s['cpu_temp']} rss {r:.0f} MB state {s['state']}", flush=True)
    s = A.status()
    rec_path, rec_csv = s["rec_path"], s["rec_csv"]
    A.post("/record", {"on": False})
    rec_s = time.monotonic() - t_rec
    A.post("/cmd", {"action": "stop"})
    rss_end = rss_mb(a.pid)
    plog.stop()
    live.close()
    st.close()
    A.stop_heartbeat()

    # per-tick ages from the recorded run CSV
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "logs", rec_csv)) as f:
        rows = list(csv.DictReader(f))
    num = lambda k: [float(r[k]) for r in rows if r[k] not in ("-", "")]
    lane_age, det_age = num("lane_age"), num("det_age")
    states = {}
    for r in rows:
        states[r["state"]] = states.get(r["state"], 0) + 1
    exc = [r for r in rows if r["state"] == "EXCEPTION"]
    vid = os.path.join(here, "videos", rec_path)
    cap = cv2.VideoCapture(vid)
    vframes = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    tb = 0
    if a.server_log and os.path.exists(a.server_log):
        with open(a.server_log) as f:
            tb = f.read().count("Traceback")

    def agestat(v):
        if not v:
            return None
        return {"mean": round(statistics.mean(v), 3), "max": round(max(v), 3),
                "p99": round(sorted(v)[int(0.99 * (len(v) - 1))], 3)}

    def mmm(k):
        v = [x[k] for x in samples if x[k] is not None]
        if not v:
            return None
        return {"mean": round(statistics.mean(v), 2), "min": min(v), "max": max(v)}
    temp_curve = [x["cpu_temp"] for x in samples if int(x["t"]) % 60 < 5]
    res = {"minutes": a.minutes, "samples": len(samples), "ctrl_hz": mmm("ctrl_hz"), "lane_fps": mmm("lane_fps"),
           "det_fps": mmm("det_fps"), "view_fps": mmm("view_fps"),
           "lane_age_tick": agestat(lane_age), "det_age_tick": agestat(det_age),
           "csv_ticks": len(rows), "csv_ticks_expected": round(rec_s * 20), "states": states,
           "temp_curve_per_min": temp_curve, "temp_max": max(x["cpu_temp"] for x in samples),
           "throttled_seen": sorted({x["throttled"] for x in samples}),
           "rss_start_mb": samples[0]["rss_mb"], "rss_1min_mb": round(rss_1min, 1), "rss_end_mb": round(rss_end, 1),
           "rss_growth_pct": round(100 * (rss_end - rss_1min) / rss_1min, 2),
           "exceptions_csv": len(exc), "tracebacks_in_server_log": tb, "failsafe_stops": stops,
           "video": rec_path, "video_frames": vframes, "video_frames_expected": round(rec_s * 10),
           "stream_frames_received": st.frames,
           "power_log": {"n": len(plog.rows), "throttled_values": sorted({r[2] for r in plog.rows}),
                         "ext5v_min": min((float(r[3]) for r in plog.rows if r[3].replace(".", "").isdigit()), default=None),
                         "ext5v_max": max((float(r[3]) for r in plog.rows if r[3].replace(".", "").isdigit()), default=None)}}
    with open(os.path.join(a.out, "summary.json"), "w") as f:
        json.dump(res, f, indent=1)
    with open(os.path.join(a.out, "samples.json"), "w") as f:
        json.dump(samples, f)
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
