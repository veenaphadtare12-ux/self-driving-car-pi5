#!/usr/bin/env python3
"""Demo-day pre-flight checks (UFLD Part 7). Called by preflight.sh; prints PASS / WARN / FAIL per check.
No motor is ever driven. Exit code 0 = no FAIL.

  python preflight_checks.py [--skip-fps] [--fps-seconds 10]
"""
import argparse
import hashlib
import os
import shutil
import subprocess
import sys
import time

import config

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = []


def report(name, status, detail=""):
    RESULTS.append((name, status, detail))
    col = {"PASS": "\033[32m", "WARN": "\033[33m", "FAIL": "\033[31m"}.get(status, "")
    print(f"{col}{status:4s}\033[0m  {name:28s} {detail}", flush=True)


def check(name):
    def deco(fn):
        def run(*a, **k):
            try:
                fn(*a, **k)
            except Exception as e:                               # noqa: BLE001
                report(name, "FAIL", f"{type(e).__name__}: {e}")
        return run
    return deco


@check("camera detected")
def camera_detected():
    out = subprocess.run(["rpicam-hello", "--list-cameras"], capture_output=True, text=True, timeout=20)
    txt = out.stdout + out.stderr
    import re
    ok = "No cameras" not in txt and ("ov5647" in txt or re.search(r"^\s*0 :", txt, re.M) is not None)
    report("camera detected", "PASS" if ok else "FAIL",
           next((l.strip() for l in txt.splitlines() if l.strip().startswith("0 :")), txt.strip()[:80]))


@check("camera colours")
def camera_colours():
    import numpy as np
    from camera import PiCamera
    cam = PiCamera()
    try:
        for _ in range(15):                    # let auto exposure / white balance settle
            img = cam.read()
    finally:
        cam.close()
    m = img.reshape(-1, 3).mean(0)
    sd = img.reshape(-1, 3).std(0)
    ratio = (m.max() + 1) / (m.min() + 1)
    ok = img.shape[:2] == (config.CAM_HEIGHT, config.CAM_WIDTH) and 15 < m.mean() < 240 and sd.min() > 5 and ratio < 2.5
    import cv2
    cv2.imwrite(os.path.join(HERE, "logs", "preflight_frame.jpg"), img)
    report("camera colours", "PASS" if ok else "WARN",
           f"{img.shape[1]}x{img.shape[0]} mean B,G,R {m.round(0).tolist()} std {sd.round(0).tolist()} "
           f"(frame: logs/preflight_frame.jpg; check a red object looks red)")


@check("distance sensors")
def sensors():
    from sensors import Sensors
    s = Sensors(use_gyro=False).start()
    try:
        t_end = time.monotonic() + 15
        tof = fl = fr = None
        while time.monotonic() < t_end and (tof is None or fl is None or fr is None):
            tof = s.tof_min_mm() if tof is None else tof
            a, b = s.ultrasonic_mm()
            fl, fr = fl if fl is not None else a, fr if fr is not None else b
            time.sleep(0.1)
    finally:
        s.close()
    for name, v in (("ToF", tof), ("ultrasonic left", fl), ("ultrasonic right", fr)):
        report(name, "PASS" if v is not None else "FAIL", "-" if v is None else f"{v:.0f} mm")


@check("IMU")
def imu():
    from smbus2 import SMBus
    with SMBus(config.I2C_BUS) as bus:
        who = bus.read_byte_data(config.MPU6050_ADDR, 0x75)
    report("IMU", "PASS" if who in (0x68, 0x70, 0x72, 0x98) else "WARN", f"MPU6050 WHO_AM_I 0x{who:02x}")


@check("model checksums")
def checksums():
    bad = []
    with open(config.UFLD_SHA256) as f:
        for line in f:
            if not line.strip():
                continue
            digest, name = line.split()
            p = os.path.join(os.path.dirname(config.UFLD_SHA256), name.lstrip("*"))
            h = hashlib.sha256()
            with open(p, "rb") as g:
                for chunk in iter(lambda: g.read(1 << 20), b""):
                    h.update(chunk)
            if h.hexdigest() != digest:
                bad.append(name)
    report("model checksums", "FAIL" if bad else "PASS", f"mismatch: {bad}" if bad else "UFLD models + anchors match SHA256SUMS")


@check("models load")
def models_load():
    import numpy as np
    from lanes import make_lane
    from obstacle_yolo import ObstacleDetector
    img = np.zeros((config.CAM_HEIGHT, config.CAM_WIDTH, 3), np.uint8)
    for m in config.UFLD_MODELS:
        make_lane("ufld", model=m).process(img)
    make_lane("hough").process(img)
    ObstacleDetector(threads=config.YOLO_THREADS).detect(img)
    report("models load", "PASS", f"UFLD {list(config.UFLD_MODELS)}, Hough, YOLO NCNN")


def fps_test(seconds):
    for prof in config.RUNTIME_PROFILES:
        _fps_profile(prof, seconds)


def _fps_profile(prof, seconds):
    name = f"FPS {prof}"
    try:
        p = config.RUNTIME_PROFILES[prof]
        js = os.path.join(HERE, "logs", f"preflight_fps_{prof}.json")
        cmd = [sys.executable, os.path.join(HERE, "bench_runtime.py"), "--profile", prof,
               "--spin", str(int(config.UFLD_ALLOW_SPINNING)), "--duration", str(seconds), "--warmup", "4", "--json", js]
        if os.path.exists(js):
            os.remove(js)
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=seconds + 120)
        import json
        if not os.path.exists(js):
            return report(name, "FAIL", f"benchmark did not finish: {(out.stderr or out.stdout).strip()[-120:]}")
        with open(js) as f:
            r = json.load(f)[0]
        yolo = p.get("yolo", True)
        lane_min = config.FAILSAFE_UFLD_MIN_FPS + config.PREFLIGHT_LANE_MARGIN_FPS
        ok = r["lane_fps"] >= lane_min and r["overruns"] == 0 and not r["errors"] and \
            (not yolo or r["det_fps"] >= config.PREFLIGHT_MIN_DET_FPS)
        det = (f"det {r['det_fps']} (need >= {config.PREFLIGHT_MIN_DET_FPS:g}), " if yolo else "no YOLO, ")
        report(name, "PASS" if ok else "FAIL",
               f"{seconds}s {p['ufld_model']} {p['ufld_threads']}t: lane {r['lane_fps']} (need >= {lane_min:g}), {det}"
               f"control {r['ctrl_hz']} Hz, lane age max {r['lane_age']['max']} s, {r['temp_max']} C")
    except Exception as e:                                       # noqa: BLE001
        report(name, "FAIL", f"{type(e).__name__}: {e}")


@check("CPU temperature")
def temperature():
    from runtime import read_cpu_temp
    t = read_cpu_temp()
    st = "PASS" if t < 70 else ("WARN" if t < config.FAILSAFE_TEMP_SLOW_C else "FAIL")
    report("CPU temperature", st, f"{t:.1f} C (half speed at {config.FAILSAFE_TEMP_SLOW_C:g}, stop at {config.FAILSAFE_TEMP_STOP_C:g})")


@check("power / throttling")
def throttling():
    from runtime import read_throttled
    th = read_throttled()
    if th is None:
        return report("power / throttling", "WARN", "vcgencmd not available")
    now, past = th & 0xF, th & 0xF0000
    st = "FAIL" if now else ("WARN" if past else "PASS")
    report("power / throttling", st, f"get_throttled={hex(th)}" + (" (under-voltage/throttling NOW)" if now else
                                                                     " (happened since boot)" if past else ""))


@check("disk space")
def disk():
    free = shutil.disk_usage(HERE).free / 1e9
    report("disk space", "PASS" if free >= 2 else "FAIL", f"{free:.1f} GB free (videos ~10-20 MB/min)")


@check("network")
def network():
    from run_threaded import local_ip
    ip = local_ip()
    report("network", "PASS" if not ip.startswith("127.") else "FAIL", f"page: http://{ip}:8001/")


@check("fans")
def fans():
    from fans import Fans
    f = Fans()
    f.on()
    time.sleep(1.5)
    ok = getattr(f, "ok", True)
    f.cleanup()
    report("fans", "PASS" if ok else "FAIL", "relay switched ON for 1.5 s then OFF - listen for the fans")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-fps", action="store_true")
    ap.add_argument("--fps-seconds", type=int, default=10)
    a = ap.parse_args()
    os.makedirs(os.path.join(HERE, "logs"), exist_ok=True)
    print(f"=== pre-flight {time.strftime('%Y-%m-%d %H:%M:%S')} (no motor is driven) ===")
    camera_detected()
    camera_colours()
    sensors()
    imu()
    checksums()
    models_load()
    if not a.skip_fps:
        fps_test(a.fps_seconds)
    temperature()
    throttling()
    disk()
    network()
    fans()
    n_fail = sum(s == "FAIL" for _, s, _ in RESULTS)
    n_warn = sum(s == "WARN" for _, s, _ in RESULTS)
    print(f"=== {len(RESULTS) - n_fail - n_warn} PASS, {n_warn} WARN, {n_fail} FAIL ===")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
