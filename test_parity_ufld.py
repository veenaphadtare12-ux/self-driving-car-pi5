#!/usr/bin/env python3
"""UFLD Part 2 parity: does pi_deploy's lanes/ufld.py give the same accuracy as car_project?

  python test_parity_ufld.py                      # all 578 s04 val frames + 150-frame subset, v2_same
  python test_parity_ufld.py --model v2_plus
  python test_parity_ufld.py --no-ref             # skip the point-by-point car_project comparison

Metric = car_project/scripts/benchmark_lane_model.py accuracy() (= the training notebook's evaluate()),
copied unchanged; only the detector is pi_deploy's UfldModel. Reference: 0.9373 point accuracy on all
578 s04 val frames (car_project PROGRESS.md, PIL resize). Pass threshold: >= 0.930.

With the reference check on, car_project's own LaneDetector also runs on every frame and the two
outputs are compared point by point (same rows, max |dx|).
No camera, no motors, no sensors.
"""
import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

import config
from lanes.ufld import UfldModel

CAR_PROJECT = os.path.expanduser("~/car_project")
VAL_LABELS = os.path.join(CAR_PROJECT, "export", "lane_v2", "label_val.json")
THRESHOLD = 0.930
REFERENCE = 0.9373
SUBSET_N = 150


def load_labels():
    return [json.loads(l) for l in open(VAL_LABELS) if l.strip()]


def img_path(L):
    return os.path.join(CAR_PROJECT, "dataset", "lane", L["raw_file"].split("/")[1], os.path.basename(L["raw_file"]))


class Acc:
    """benchmark_lane_model.accuracy(), split so it can be fed frame by frame (logic unchanged)."""

    def __init__(self, anchors, thr=20):
        self.anchors, self.thr = anchors, thr
        self.correct = self.total = self.tp = self.fp = self.fn = 0
        self.errs = []

    def add(self, L, pred, h):
        for i in range(2):
            xs = L["lanes"][i]
            gt_vis = sum(x >= 0 for x in xs) >= 3
            p = pred[i] if i < len(pred) else {}
            if gt_vis and p:
                self.tp += 1
            elif gt_vis:
                self.fn += 1
            elif p:
                self.fp += 1
            if not gt_vis:
                continue
            v = [(a, b) for a, b in zip(L["h_samples"], xs) if b >= 0]

            def gt_x(y):
                if len(v) < 2 or y < v[0][0] or y > v[-1][0]:
                    return None
                return float(np.interp(y, [a for a, _ in v], [b for _, b in v]))
            for y, px in p.items():
                g = gt_x(y)
                if g is None:
                    continue
                self.total += 1
                e = abs(px - g)
                self.errs.append(e)
                self.correct += e < self.thr
            for y in [int(h * r / 288) - 1 for r in self.anchors]:
                if gt_x(y) is not None and y not in p:
                    self.total += 1

    def result(self, n):
        return {"n_val": n, "point_accuracy": self.correct / max(self.total, 1),
                "mean_abs_error_px": float(np.mean(self.errs)) if self.errs else float("nan"),
                "lane_precision": self.tp / max(self.tp + self.fp, 1),
                "lane_recall": self.tp / max(self.tp + self.fn, 1)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=config.UFLD_MODEL, choices=sorted(config.UFLD_MODELS))
    ap.add_argument("--threads", type=int, default=config.UFLD_THREADS)
    ap.add_argument("--no-ref", action="store_true", help="skip car_project LaneDetector comparison")
    ap.add_argument("--json", default=None)
    a = ap.parse_args()

    path = config.UFLD_MODELS[a.model]
    det = UfldModel(path, threads=a.threads, resize=config.UFLD_RESIZE)
    ref = None
    if not a.no_ref:
        sys.path.insert(0, os.path.join(CAR_PROJECT, "scripts"))
        from lane_infer import LaneDetector
        ref = LaneDetector(path, threads=a.threads)

    labels = load_labels()
    subset = set(np.linspace(0, len(labels) - 1, SUBSET_N).round().astype(int).tolist())
    acc_all, acc_sub = Acc(det.anchors), Acc(det.anchors)
    max_dx, row_mismatch, t_det = 0.0, 0, []
    print(f"model {os.path.basename(path)} | {len(labels)} s04 val frames (subset {len(subset)} evenly spaced) | "
          f"threads {a.threads} | reference check {'on' if ref else 'off'}")
    t0 = time.time()
    for k, L in enumerate(labels):
        img = cv2.imread(img_path(L))
        h = img.shape[0]
        t = time.perf_counter()
        pred, _conf = det.detect(img, h=h, w=img.shape[1])
        t_det.append(time.perf_counter() - t)
        acc_all.add(L, pred, h)
        if k in subset:
            acc_sub.add(L, pred, h)
        if ref is not None:
            rp = ref.detect(img)
            for i in range(2):
                a_, b_ = pred[i], rp[i] if i < len(rp) else {}
                if set(a_) != set(b_):
                    row_mismatch += 1
                for y in set(a_) & set(b_):
                    max_dx = max(max_dx, abs(a_[y] - b_[y]))
        if (k + 1) % 100 == 0:
            print(f"  {k + 1}/{len(labels)}  {time.time() - t0:.0f} s")

    r_all, r_sub = acc_all.result(len(labels)), acc_sub.result(len(subset))
    fmt = lambda r: {k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()}
    print("ALL   :", fmt(r_all))
    print("SUB150:", fmt(r_sub))
    print(f"detect median {np.median(t_det) * 1000:.0f} ms ({1 / np.median(t_det):.1f} fps, pipeline only)")
    ok = r_all["point_accuracy"] >= THRESHOLD and r_sub["point_accuracy"] >= THRESHOLD
    if ref is not None:
        print(f"vs car_project LaneDetector: lane row-set mismatches {row_mismatch}, max |dx| {max_dx:.6f} px")
        ok = ok and row_mismatch == 0 and max_dx < 0.01
    print(f"reference {REFERENCE} (car_project, all 578) | threshold {THRESHOLD} -> {'PASS' if ok else 'FAIL'}")
    if a.json:
        json.dump({"model": a.model, "all": r_all, "subset150": r_sub, "ref_row_mismatch": row_mismatch,
                   "ref_max_dx": max_dx, "pass": ok}, open(a.json, "w"), indent=1)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
