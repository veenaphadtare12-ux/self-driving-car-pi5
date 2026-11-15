#!/usr/bin/env python3
"""UFLD Part 3 benchmark matrix: threaded runtime, DRY RUN, UFLD + YOLO + sensors + fixed-rate loop.

  python bench_runtime.py --source replay/s04_640x480.avi          # replay (30 fps, like the camera)
  python bench_runtime.py                                           # live camera
  python bench_runtime.py --ufld 3 --ncnn 1 --every 3 --duration 20 # one setting
  python bench_runtime.py --spin 0 1                                # onnxruntime spinning off / on

Per setting: warm-up, then `duration` s measured. Reports lane / detection FPS (results per second over
the window), control rate + period jitter + overruns, lane and detection age at the control tick
(what decide() sees: mean / max) and latency (capture -> result), CPU busy sum (proof of parallelism:
lane_fps*lane_proc + det_fps*det_proc > 1 means the two models ran at the same time), CPU temperature,
throttled flags. Motors: DryRunMotors only.
"""
import argparse
import itertools
import json
import os
import sys
import time

import config
from runtime import CarRuntime, SysMonitor

TARGETS = {"lane_fps": 4.0, "det_fps": 5.0, "max_age": 0.4, "temp": 80.0}


def meets(r):
    fails = []
    if r["lane_fps"] < TARGETS["lane_fps"]:
        fails.append("lane_fps")
    if r["det_fps"] < TARGETS["det_fps"]:
        fails.append("det_fps")
    if r["lane_age"]["max"] is None or r["lane_age"]["max"] >= TARGETS["max_age"]:
        fails.append("lane_age")
    if r["det_age"]["max"] is None or r["det_age"]["max"] >= TARGETS["max_age"]:
        fails.append("det_age")
    if r["temp_max"] is not None and r["temp_max"] >= TARGETS["temp"]:
        fails.append("temp")
    if int(r["throttled_seen"], 16) & 0xF:
        fails.append("throttled")
    if not r["loop_steady"]:
        fails.append("loop")
    return fails


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default=None, help="replay folder/video (default: live camera)")
    ap.add_argument("--ufld", type=int, nargs="+", default=[2, 3, 4])
    ap.add_argument("--ncnn", type=int, nargs="+", default=[1, 2])
    ap.add_argument("--every", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--spin", type=int, nargs="+", default=[int(config.UFLD_ALLOW_SPINNING)])
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--warmup", type=float, default=6.0)
    ap.add_argument("--no-sensors", action="store_true")
    ap.add_argument("--model", default=config.UFLD_MODEL)
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    from fans import Fans
    from lanes.ufld import UfldLane
    from run_threaded import make_detector, open_source
    fans = Fans()
    fans.on()
    sensors = None
    if not a.no_sensors:
        from sensors import Sensors
        sensors = Sensors(use_gyro=False).start()
    cam = open_source(a.source)
    mon = SysMonitor()
    mon.start()
    dets = {}
    rows = []
    mode = "replay" if a.source else "live"
    print(f"mode {mode} | {a.duration:.0f} s per setting | sensors {'on' if sensors else 'off'} | "
          f"control {config.CONTROL_HZ} Hz | SoC {mon.temp_c} C throttled {hex(mon.throttled or 0)}")
    hdr = (f"{'ufld':>4} {'ncnn':>4} {'N':>2} {'spin':>4} | {'lane':>5} {'det':>5} {'ctrl':>5} {'p.max':>6} {'ovr':>3} | "
           f"{'l.age':>5} {'l.max':>5} {'d.age':>5} {'d.max':>5} | {'busy':>4} {'temp':>5} {'thr':>4} | fails")
    print(hdr)
    rt = None
    try:
        for spin, ut, nt, every in itertools.product(a.spin, a.ufld, a.ncnn, a.every):
            config.UFLD_ALLOW_SPINNING = bool(spin)
            lane = UfldLane(model=a.model, threads=ut)
            if nt not in dets:
                dets[nt] = make_detector(nt)
            mon.temp_max = mon.temp_c
            mon.throttled_seen = 0
            rt = CarRuntime(cam, lane, dets[nt], sensors, lane_backend="ufld", yolo_every_n=every,
                            running=True, monitor=mon, phase="P7")
            rt.start_threads()
            rt.run(duration=a.warmup)
            rt.reset_stats()
            n_l0, n_d0, t0 = rt.lane_worker.n, rt.det_worker.n, time.monotonic()
            mon.temp_max = mon.temp_c
            mon.throttled_seen = mon.throttled or 0
            rt.run(duration=a.duration)
            dt = time.monotonic() - t0
            s = rt.bench_summary()
            s.update(mode=mode, ufld_threads=ut, ncnn_threads=nt, every=every, spin=spin,
                     lane_fps=round((rt.lane_worker.n - n_l0) / dt, 2), det_fps=round((rt.det_worker.n - n_d0) / dt, 2))
            lp, dp = (s["lane_proc_ms"]["mean"] or 0) / 1000, (s["det_proc_ms"]["mean"] or 0) / 1000
            s["busy"] = round(s["lane_fps"] * lp + s["det_fps"] * dp, 2)
            s["loop_steady"] = (s["ctrl_hz"] is not None and abs(s["ctrl_hz"] - config.CONTROL_HZ) < 0.2
                                and s["period_ms"]["max"] < 2000.0 / config.CONTROL_HZ and s["overruns"] == 0)
            s["fails"] = meets(s)
            rt.close()
            rows.append(s)
            print(f"{ut:>4} {nt:>4} {every:>2} {spin:>4} | {s['lane_fps']:5.2f} {s['det_fps']:5.2f} {s['ctrl_hz']:5.2f} "
                  f"{s['period_ms']['max']:6.1f} {s['overruns']:>3} | {s['lane_age']['mean']:5.3f} {s['lane_age']['max']:5.3f} "
                  f"{s['det_age']['mean']:5.3f} {s['det_age']['max']:5.3f} | {s['busy']:4.2f} {s['temp_max']:5.1f} "
                  f"{s['throttled_seen']:>4} | {','.join(s['fails']) or 'MEETS ALL'}", flush=True)
            if s["errors"]:
                print("   errors:", s["errors"])
            if a.json:
                with open(a.json, "w") as f:
                    json.dump(rows, f, indent=1)
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        if rt is not None:
            rt.close()
        mon.stop()
        cam.close()
        if sensors:
            sensors.close()
        fans.cleanup()
    ok = [r for r in rows if not r["fails"]]
    print(f"{len(ok)}/{len(rows)} settings meet every target")
    return 0


if __name__ == "__main__":
    sys.exit(main())
