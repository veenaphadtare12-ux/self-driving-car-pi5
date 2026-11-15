"""
Calibrate the traffic-light / stop-sign thresholds on the Pi (or on photos on the laptop).

Put the car where it should STOP for a red light (or a stop sign) and run:

    python calibrate_signals.py                    # picamera2, Ctrl+C or q to finish
    python calibrate_signals.py --seconds 20
    python calibrate_signals.py --source photo.jpg # laptop

For every light_red / light_green / stop_sign (and every obstacle) it prints:
conf, box height ratio, box center x, and whether the box is inside SIGNAL_REGION_X / the corridor.
At the end it prints a per-class summary and suggested config values:
  * RED_LIGHT_HEIGHT_RATIO: red boxes at least this tall -> STOP_RED (smaller -> CAUTION)
  * STOP_SIGN_HEIGHT_RATIO
  * SIGNAL_REGION_X: should contain the center x of the lights
Move the car closer / further and re-run to see how height changes with distance.
"""
import argparse
import statistics
import time

import config
from camera import open_camera
from decision import in_corridor
from fans import Fans
from keys import KeyReader
from obstacle_yolo import ObstacleDetector


def in_signal_region(d):
    lo, hi = config.SIGNAL_REGION_X
    return lo <= d["cx_ratio"] <= hi


def describe(d):
    tags = []
    if d["cls"] in (config.RED_LIGHT_CLASS, config.GREEN_LIGHT_CLASS):
        tags.append("in-region" if in_signal_region(d) else "OUTSIDE-region")
        if d["conf"] < config.SIGNAL_CONF:
            tags.append(f"conf<{config.SIGNAL_CONF}")
        if d["cls"] == config.RED_LIGHT_CLASS and in_signal_region(d) and d["conf"] >= config.SIGNAL_CONF:
            tags.append("-> STOP_RED" if d["box_height_ratio"] >= config.RED_LIGHT_HEIGHT_RATIO else "-> CAUTION")
    elif d["cls"] == config.STOP_SIGN_CLASS:
        ok = d["box_height_ratio"] >= config.STOP_SIGN_HEIGHT_RATIO and d["conf"] >= config.STOP_SIGN_CONF
        tags.append("-> STOP_SIGN (after 3 checks)" if ok else "too small/low conf")
    elif d["cls"] in config.OBSTACLE_CLASSES:
        tags.append("IN corridor" if in_corridor(d) else "outside corridor")
    return (f"{d['cls']:12s} conf {d['conf']:.2f}  h {d['box_height_ratio']:.3f}  "
            f"center_x {d['cx_ratio']:.3f}  bottom {d['bottom_ratio']:.2f}  {' '.join(tags)}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", default=None)
    p.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until q/Ctrl+C)")
    a = p.parse_args()

    fans = Fans()
    fans.on()
    cam = None
    stats = {}
    frames = 0
    try:
        det = ObstacleDetector()
        cam = open_camera(a.source)
        single = a.source is not None and a.source.lower().endswith((".jpg", ".jpeg", ".png", ".bmp"))
        print(f"SIGNAL_REGION_X={config.SIGNAL_REGION_X}  RED_LIGHT_HEIGHT_RATIO={config.RED_LIGHT_HEIGHT_RATIO}  "
              f"STOP_SIGN_HEIGHT_RATIO={config.STOP_SIGN_HEIGHT_RATIO}  CORRIDOR_X={config.CORRIDOR_X}")
        t_end = time.monotonic() + a.seconds if a.seconds else None
        with KeyReader() as keys:
            while True:
                frame = cam.read()
                if frame is None:
                    break
                dets = det.detect(frame)
                frames += 1
                print(f"[{frames:4d}] " + ("  |  ".join(describe(d) for d in dets) if dets else "-"))
                for d in dets:
                    s = stats.setdefault(d["cls"], {"h": [], "x": [], "conf": []})
                    s["h"].append(d["box_height_ratio"])
                    s["x"].append(d["cx_ratio"])
                    s["conf"].append(d["conf"])
                if single or keys.get() == "q" or (t_end and time.monotonic() > t_end):
                    break
    except KeyboardInterrupt:
        print()
    finally:
        if cam is not None:
            cam.close()
        fans.cleanup()

    print(f"\n=== Summary over {frames} frames ===")
    print(f"{'class':12s} {'n':>4s}  {'h min':>6s} {'h med':>6s} {'h max':>6s}  {'x min':>6s} {'x med':>6s} {'x max':>6s}  conf med")
    for cls, s in sorted(stats.items()):
        print(f"{cls:12s} {len(s['h']):4d}  {min(s['h']):6.3f} {statistics.median(s['h']):6.3f} {max(s['h']):6.3f}  "
              f"{min(s['x']):6.3f} {statistics.median(s['x']):6.3f} {max(s['x']):6.3f}  {statistics.median(s['conf']):.2f}")
    if config.RED_LIGHT_CLASS in stats:
        s = stats[config.RED_LIGHT_CLASS]
        print(f"\nIf this is the stop position for the red light:\n"
              f"  RED_LIGHT_HEIGHT_RATIO = {0.95 * statistics.median(s['h']):.3f}   (95 % of the median height)\n"
              f"  red center x {min(s['x']):.2f}..{max(s['x']):.2f}: SIGNAL_REGION_X = {config.SIGNAL_REGION_X} "
              f"{'OK' if config.SIGNAL_REGION_X[0] <= min(s['x']) else '-> lower the left edge'}")
    if config.STOP_SIGN_CLASS in stats:
        s = stats[config.STOP_SIGN_CLASS]
        print(f"\nIf this is the stop position for the stop sign:\n"
              f"  STOP_SIGN_HEIGHT_RATIO = {0.95 * statistics.median(s['h']):.3f}")


if __name__ == "__main__":
    main()
