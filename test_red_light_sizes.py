#!/usr/bin/env python3
"""Red / green light detection rate by box size, and how often the 3-in-a-row rule would fail.

  python test_red_light_sizes.py [--json out.json]

Frames: ~/car_project/export/obstacles_v1 (train+valid) s5_signal_red_* (red in view) and
s06_signal_green_* (green), sorted by photo number. NCNN through pi_deploy's ObstacleDetector, scanned
at conf 0.10. The light's size in a frame = height ratio of the best box of the right colour at conf >= 0.10;
when there is no box at all, the size is interpolated from the nearest photos (+-5 numbers) that have one
(otherwise "unknown").

Buckets (box height / frame height): small < 0.10, medium 0.10-0.15, large >= 0.15 (= RED_LIGHT_HEIGHT_RATIO,
the size at which decide() stops for red, i.e. the stopping distance).

Rule check per frame (decide()): red box with conf >= SIGNAL_CONF, centre x in SIGNAL_REGION_X;
STOP_RED additionally needs that box height >= RED_LIGHT_HEIGHT_RATIO and SIGNAL_CONFIRM_CHECKS (3)
consecutive YOLO checks. Reported for the large bucket:
  p        = per-check pass rate
  iid      = 1 - p^3: a 3-check window fails if any check misses (checks independent)
  photos   = fraction of 3 consecutive large-bucket photos (by number) that do NOT all pass
Caveat: photos were taken ~2/s while moving the car by hand, and were most likely in the training export.
"""
import argparse
import bisect
import glob
import json
import os
import re

import cv2

import config
from obstacle_yolo import ObstacleDetector

DATA = os.path.expanduser("~/car_project/export/obstacles_v1")
LOW = 0.10
BUCKETS = (("small", 0.0, 0.10), ("medium", 0.10, config.RED_LIGHT_HEIGHT_RATIO),
           ("large", config.RED_LIGHT_HEIGHT_RATIO, 9.0))


def frames(prefix):
    fs = glob.glob(os.path.join(DATA, "*", "images", f"{prefix}_*.jpg"))
    num = lambda f: int(re.search(r"_(\d+)\.jpg$", f).group(1))
    return sorted(((num(f), f) for f in fs))


def bucket(h):
    for name, lo, hi in BUCKETS:
        if lo <= h < hi:
            return name
    return "unknown"


def analyse(det, prefix, want):
    rows = []
    for n, f in frames(prefix):
        d = [x for x in det.detect(cv2.imread(f)) if x["cls"] == want]
        best = max(d, key=lambda x: x["conf"], default=None)
        rule = [x for x in d if x["conf"] >= config.SIGNAL_CONF
                and config.SIGNAL_REGION_X[0] <= x["cx_ratio"] <= config.SIGNAL_REGION_X[1]]
        rows.append({"n": n, "file": os.path.basename(f),
                     "conf": best["conf"] if best else 0.0,
                     "h": best["box_height_ratio"] if best else None,
                     "cx": best["cx_ratio"] if best else None,
                     "det04": any(x["conf"] >= config.YOLO_CONF for x in d),
                     "det05": any(x["conf"] >= config.SIGNAL_CONF for x in d),
                     "rule": bool(rule),
                     "rule_h": max((x["box_height_ratio"] for x in rule), default=0.0)})
    # sizes for frames without any box: nearest photos with a box, within +-5 numbers
    known = [(r["n"], r["h"]) for r in rows if r["h"] is not None]
    ks = [k for k, _ in known]
    for r in rows:
        r["h_est"], r["h_src"] = r["h"], "box"
        if r["h"] is None:
            i = bisect.bisect_left(ks, r["n"])
            near = [known[j] for j in (i - 1, i) if 0 <= j < len(known) and abs(known[j][0] - r["n"]) <= 5]
            if near:
                r["h_est"], r["h_src"] = sum(h for _, h in near) / len(near), "neighbours"
            else:
                r["h_est"], r["h_src"] = None, "unknown"
        r["bucket"] = "unknown" if r["h_est"] is None else bucket(r["h_est"])
        r["stop_ok"] = r["rule"] and r["rule_h"] >= config.RED_LIGHT_HEIGHT_RATIO
    table = {}
    for name in [b[0] for b in BUCKETS] + ["unknown"]:
        sel = [r for r in rows if r["bucket"] == name]
        if not sel:
            continue
        k = len(sel)
        table[name] = {"frames": k, "det@0.4": sum(r["det04"] for r in sel), "det@0.5": sum(r["det05"] for r in sel),
                       "rule(conf+region)": sum(r["rule"] for r in sel),
                       "rate@0.4": round(sum(r["det04"] for r in sel) / k, 3),
                       "rate_rule": round(sum(r["rule"] for r in sel) / k, 3),
                       "size_from_neighbours": sum(r["h_src"] == "neighbours" for r in sel)}
    large = [r for r in rows if r["bucket"] == "large"]
    out = {"frames": len(rows), "by_size": table}
    if large:
        key = "stop_ok" if want == config.RED_LIGHT_CLASS else "rule"
        p = sum(r[key] for r in large) / len(large)
        wins = [rows[i:i + 3] for i in range(len(rows) - 2)]
        wins = [w for w in wins if all(r["bucket"] == "large" for r in w) and w[2]["n"] - w[0]["n"] <= 6]
        fail_w = sum(not all(r[key] for r in w) for w in wins)
        out["three_in_a_row_large"] = {
            "per_check_pass": round(p, 3), "iid_window_fail": round(1 - p ** 3, 3),
            "photo_windows": len(wins), "photo_windows_failing": fail_w,
            "photo_window_fail_rate": round(fail_w / len(wins), 3) if wins else None,
            "out_of_region_large": sum(r["det05"] and not r["rule"] for r in large)}
    out["rows"] = rows
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    det = ObstacleDetector(conf=LOW)
    res = {}
    for prefix, want in (("s5_signal_red", config.RED_LIGHT_CLASS), ("s06_signal_green", config.GREEN_LIGHT_CLASS)):
        r = analyse(det, prefix, want)
        res[prefix] = r
        print(f"== {prefix} ({want}), {r['frames']} photos")
        print(f"   {'size':8s} {'frames':>6} {'det@0.4':>8} {'rate':>6} {'rule':>5} {'rate':>6} {'size est.':>9}")
        for name, t in r["by_size"].items():
            print(f"   {name:8s} {t['frames']:6d} {t['det@0.4']:8d} {t['rate@0.4']:6.2f} {t['rule(conf+region)']:5d} "
                  f"{t['rate_rule']:6.2f} {t['size_from_neighbours']:9d}")
        if "three_in_a_row_large" in r:
            print("   3-in-a-row at stopping size:", r["three_in_a_row_large"])
    if a.json:
        with open(a.json, "w") as f:
            json.dump(res, f, indent=1)


if __name__ == "__main__":
    main()
