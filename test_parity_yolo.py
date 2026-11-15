#!/usr/bin/env python3
"""UFLD Part 2 parity, YOLO half: is the NCNN export safe, and does it match the original model?

  python test_parity_yolo.py                       # red-never-green check on the signal sessions
  python test_parity_yolo.py --ref best.pt         # + NCNN vs Ultralytics PyTorch, box by box

1) Red never green (always). Frames come from ~/car_project/export/obstacles_v1 (train + valid), scene
   known from the file name: s5_signal_red_* = red light in view, s06_signal_green_* = green light.
   NCNN runs through pi_deploy's ObstacleDetector (imgsz YOLO_IMGSZ) at conf 0.10 so the margin is visible.
   FAIL if any red frame gets light_green at >= YOLO_CONF (any box the pipeline would see).
   Also reported: green frames with light_red (safe direction: car stops), red/green detection rate,
   highest light_green conf on red frames (margin).
   Caveat: these images were most likely in the Roboflow training export, so this checks the export
   and the wrapper, not generalisation to new scenes.

2) NCNN vs reference (only with --ref, a .pt or .onnx of the SAME fine-tuned model): every frame through
   both, boxes matched by class + IoU >= 0.5 at conf >= YOLO_CONF. Reports matched / missed / extra
   boxes, conf difference, any class change. FAIL on any red<->green change or < 95 % of reference boxes
   matched.
No camera, no motors, no sensors.
"""
import argparse
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

import config
from obstacle_yolo import ObstacleDetector

DATA = os.path.expanduser("~/car_project/export/obstacles_v1")
LOW_CONF = 0.10
RED, GREEN = config.RED_LIGHT_CLASS, config.GREEN_LIGHT_CLASS


def signal_frames():
    files = sorted(glob.glob(os.path.join(DATA, "*", "images", "*.jpg")))
    red = [f for f in files if os.path.basename(f).startswith("s5_signal_red")]
    green = [f for f in files if os.path.basename(f).startswith("s06_signal_green")]
    return red, green


def iou(a, b):
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    u = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / u if u > 0 else 0.0


def red_green_check(det, red, green):
    out = {}
    for name, files, want, bad in (("red", red, RED, GREEN), ("green", green, GREEN, RED)):
        n_want = n_bad = 0
        bad_max, bad_files = 0.0, []
        for f in files:
            d = det.detect(cv2.imread(f))
            if any(x["cls"] == want and x["conf"] >= config.YOLO_CONF for x in d):
                n_want += 1
            b = [x["conf"] for x in d if x["cls"] == bad]
            if b:
                bad_max = max(bad_max, max(b))
                if max(b) >= config.YOLO_CONF:
                    n_bad += 1
                    bad_files.append(f"{os.path.basename(f)}:{max(b):.2f}")
        out[name] = {"frames": len(files), f"with_{want}": n_want, f"with_{bad}": n_bad,
                     f"max_{bad}_conf": round(bad_max, 3), "bad_files": bad_files[:20]}
    return out


def ref_compare(det, ref_path, files):
    from ultralytics import YOLO
    ref = YOLO(ref_path, task="detect")
    names_ok = {int(k): v for k, v in ref.names.items()} == {int(k): v for k, v in det.names.items()}
    matched = missed = extra = flips = rg_flips = 0
    dconf = []
    for f in files:
        img = cv2.imread(f)
        h, w = img.shape[:2]
        a = [x for x in det.detect(img) if x["conf"] >= config.YOLO_CONF]
        r = ref.predict(img, imgsz=config.YOLO_IMGSZ, conf=config.YOLO_CONF, verbose=False)[0]
        b = [{"cls": ref.names[int(c)], "conf": float(s), "box": tuple(map(float, xy))}
             for xy, s, c in zip(r.boxes.xyxy.tolist(), r.boxes.conf.tolist(), r.boxes.cls.tolist())]
        used = set()
        for rb in b:
            best, bi = 0.0, None
            for i, nb in enumerate(a):
                if i not in used and iou(rb["box"], nb["box"]) > best:
                    best, bi = iou(rb["box"], nb["box"]), i
            if bi is not None and best >= 0.5:
                used.add(bi)
                if a[bi]["cls"] == rb["cls"]:
                    matched += 1
                    dconf.append(abs(a[bi]["conf"] - rb["conf"]))
                else:
                    flips += 1
                    rg_flips += {a[bi]["cls"], rb["cls"]} == {RED, GREEN}
            else:
                missed += 1
        extra += len(a) - len(used)
    n_ref = matched + missed + flips
    return {"ref": os.path.basename(ref_path), "class_names_equal": names_ok, "frames": len(files),
            "ref_boxes": n_ref, "matched": matched, "missed": missed, "extra_ncnn": extra,
            "class_changes": flips, "red_green_changes": rg_flips,
            "match_rate": matched / max(n_ref, 1),
            "conf_diff_median": float(np.median(dconf)) if dconf else None,
            "conf_diff_max": float(np.max(dconf)) if dconf else None}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ref", default=None, help="original fine-tuned model (.pt or .onnx) to compare against")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    det = ObstacleDetector(conf=LOW_CONF)
    red, green = signal_frames()
    print(f"NCNN {os.path.basename(config.YOLO_MODEL_PATH)} imgsz {config.YOLO_IMGSZ} | red frames {len(red)}, "
          f"green frames {len(green)} | pipeline conf {config.YOLO_CONF}, scan conf {LOW_CONF}")
    t0 = time.time()
    rg = red_green_check(det, red, green)
    for k, v in rg.items():
        print(f"  {k:5s}: {v}")
    ok = rg["red"][f"with_{GREEN}"] == 0
    res = {"red_green": rg}
    print(f"red never green: {'PASS' if ok else 'FAIL'}  ({time.time() - t0:.0f} s)")

    if a.ref:
        all_files = sorted(glob.glob(os.path.join(DATA, "valid", "images", "*.jpg")))
        cmp_ = ref_compare(det, a.ref, all_files + red + green)
        res["ref_compare"] = cmp_
        print("NCNN vs reference:", cmp_)
        c_ok = cmp_["class_names_equal"] and cmp_["red_green_changes"] == 0 and cmp_["match_rate"] >= 0.95
        print(f"NCNN vs reference: {'PASS' if c_ok else 'FAIL'}")
        ok = ok and c_ok
    else:
        print("NCNN vs reference: NOT RUN (no --ref; the fine-tuned .pt is not on the Pi)")
    res["pass"] = ok
    if a.json:
        json.dump(res, open(a.json, "w"), indent=1)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
