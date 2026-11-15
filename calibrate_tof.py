"""
Calibrate the front distance sensors on the Pi. Two steps, each a few seconds:

  1. EMPTY TRACK: nothing in front of the car for at least 1.5 m.
     Checks that the ToF center zones don't see the floor (or the car's own bumper).
  2. OBJECT AT 40 cm: a box (or your hand/leg) 400 mm in front of the bumper, centered.
     Checks the ToF center min and the ultrasonics read about 400 mm.

    python calibrate_tof.py
    python calibrate_tof.py --seconds 5 --distance 400

Prints the full 8x8 grid (median per zone), the center-zone min, both ultrasonics,
and hints for TOF_CENTER_ROWS / TOF_CENTER_COLS in config.py.
"""
import argparse
import math
import statistics
import time

import config
from sensors import Sensors


def fmt(v):
    if v is None:
        return "   -"
    return " inf" if v == math.inf else f"{v:4.0f}"


def collect(s, seconds):
    """Returns (grids, center_mins, fl_list, fr_list). grids = list of (dist, status) per ToF frame."""
    grids, centers, fl, fr = [], [], [], []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if s.tof is not None and s.tof.ok:
            z = s.tof.read_zones()
            if z is not None:
                grids.append(z)
                centers.append(s.tof.center_min_mm(z))
        a, b = s.read_ultrasonic_now()   # alternating, >= US_GAP_S apart
        fl.append(a)
        fr.append(b)
        print(f"\r  ToF center {fmt(centers[-1] if centers else None)} mm | "
              f"US FL {fmt(a)}  FR {fmt(b)} mm   ", end="", flush=True)
    print()
    return grids, centers, fl, fr


def zone_medians(grids):
    """Median of valid readings per zone (None if never valid)."""
    out = []
    for i in range(64):
        vals = []
        for dist, status, _ in grids:
            if status is not None and status[i] not in config.TOF_VALID_STATUSES:
                continue
            if config.TOF_MIN_VALID_MM <= dist[i] <= config.TOF_MAX_VALID_MM:
                vals.append(dist[i])
        out.append(statistics.median(vals) if vals else None)
    return out


def print_grid(med):
    rows, cols = set(config.TOF_CENTER_ROWS), set(config.TOF_CENTER_COLS)
    print("  8x8 median per zone (mm, '-' = no valid target, [] = center zones used for decisions)")
    print("       " + "".join(f"  c{c}  " for c in range(8)))
    for r in range(8):
        cells = []
        for c in range(8):
            v = fmt(med[r * 8 + c])
            cells.append(f"[{v}]" if r in rows and c in cols else f" {v} ")
        print(f"  r{r}  " + " ".join(cells))


def summarize(name, vals):
    good = [v for v in vals if v is not None and v != math.inf]
    n_inf = sum(1 for v in vals if v == math.inf)
    n_none = sum(1 for v in vals if v is None)
    if not good:
        return f"{name}: no distance ({n_inf} no-target, {n_none} invalid of {len(vals)})"
    return (f"{name}: median {statistics.median(good):.0f} mm, min {min(good):.0f}, max {max(good):.0f}, "
            f"spread {max(good) - min(good):.0f} ({len(good)} valid, {n_inf} no-target, {n_none} invalid)")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--seconds", type=float, default=4.0)
    p.add_argument("--distance", type=float, default=400.0, help="object distance for step 2 (mm)")
    a = p.parse_args()

    s = Sensors(use_gyro=False)    # no background thread: synchronous reads
    try:
        if not (s.tof and s.tof.ok):
            print("WARNING: ToF not available; only ultrasonics will be checked.")

        # ---- step 1 -----------------------------------------------------------
        input("\nSTEP 1: clear the track in front of the car (>= 1.5 m), then press Enter...")
        grids, centers, fl, fr = collect(s, a.seconds)
        if grids:
            med = zone_medians(grids)
            print_grid(med)
            floor_zones = [(i // 8, i % 8, med[i]) for i in range(64)
                           if med[i] is not None and med[i] < config.CAUTION_DISTANCE_MM]
            center_hits = [(r, c, v) for r, c, v in floor_zones
                           if r in config.TOF_CENTER_ROWS and c in config.TOF_CENTER_COLS]
            print("  " + summarize("ToF center min", centers))
            if center_hits:
                bad_rows = sorted({r for r, _, _ in center_hits})
                print(f"  PROBLEM: center zones see something < {config.CAUTION_DISTANCE_MM} mm on an empty track "
                      f"(rows {bad_rows}): probably the floor or the car itself.\n"
                      f"  -> remove those rows from TOF_CENTER_ROWS in config.py, or tilt the sensor up.")
            elif floor_zones:
                print(f"  OK: only non-center zones see close things (rows "
                      f"{sorted({r for r, _, _ in floor_zones})}); they are not used for decisions.")
            else:
                print("  OK: nothing closer than "
                      f"{config.CAUTION_DISTANCE_MM} mm on the empty track.")
            close = [v for v in centers if v is not None and v != math.inf and v < config.SAFETY_STOP_MM]
            if close:
                print(f"  PROBLEM: {len(close)} center readings < {config.SAFETY_STOP_MM} mm (safety stop) on an empty track.")
        print("  " + summarize("US front-left ", fl))
        print("  " + summarize("US front-right", fr))
        for name, vals in (("front-left", fl), ("front-right", fr)):
            close = [v for v in vals if v is not None and v < config.CAUTION_DISTANCE_MM]
            if close:
                print(f"  NOTE: US {name} saw < {config.CAUTION_DISTANCE_MM} mm on an empty track "
                      f"({len(close)}x, min {min(close):.0f}): floor echo or cross-talk; aim it slightly up.")

        # ---- step 2 -----------------------------------------------------------
        input(f"\nSTEP 2: put an object {a.distance:.0f} mm in front of the bumper, centered, then press Enter...")
        grids, centers, fl, fr = collect(s, a.seconds)
        if grids:
            print_grid(zone_medians(grids))
        tol = 50
        for name, vals in (("ToF center min", centers), ("US front-left ", fl), ("US front-right", fr)):
            print("  " + summarize(name, vals))
            good = [v for v in vals if v is not None and v != math.inf]
            if good:
                err = statistics.median(good) - a.distance
                verdict = "OK" if abs(err) <= tol else "CHECK (mounting offset? object not centered?)"
                print(f"      error {err:+.0f} mm vs {a.distance:.0f} -> {verdict}")
        print(f"\nDecisions use: stop < {config.STOP_DISTANCE_MM} mm, resume > "
              f"{config.STOP_DISTANCE_MM + config.RESUME_MARGIN_MM} mm, caution < {config.CAUTION_DISTANCE_MM} mm, "
              f"safety < {config.SAFETY_STOP_MM} mm. If the sensor is mounted behind the bumper, the readings "
              f"include that offset.")
    except KeyboardInterrupt:
        print()
    finally:
        s.close()


if __name__ == "__main__":
    main()
