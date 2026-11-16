#!/usr/bin/env python3
"""Stage 1 motors, ONE step per run (car on a stand, wheels lifted). Uses motors.Motors unchanged.

    python motor_step.py forward            # 1 s at --speed (default 20 % duty)
    python motor_step.py backward
    python motor_step.py left               # left side -half, right side +speed
    python motor_step.py right              # left side +speed, right side -half
    python motor_step.py right --speed 25 --seconds 1

Prints the command first, counts down 3 s (Ctrl+C aborts), drives, then active-brakes and stops.
"""
import argparse
import time

STEPS = {  # name: (left factor, right factor, what you should see)
    "forward": (1.0, 1.0, "BOTH sides spin FORWARD at about the same speed"),
    "backward": (-1.0, -1.0, "BOTH sides spin BACKWARD at about the same speed"),
    "left": (-0.5, 1.0, "LEFT side slowly BACKWARD, RIGHT side FORWARD (car would turn left)"),
    "right": (1.0, -0.5, "LEFT side FORWARD, RIGHT side slowly BACKWARD (car would turn right)"),
}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("step", choices=list(STEPS))
    p.add_argument("--speed", type=float, default=20.0)
    p.add_argument("--seconds", type=float, default=1.0)
    a = p.parse_args()
    if not 0 < a.speed <= 40 or not 0 < a.seconds <= 3:
        raise SystemExit("speed must be 0-40 and seconds 0-3 for this test")
    fl, fr, expect = STEPS[a.step]
    left, right = fl * a.speed, fr * a.speed
    print(f"STEP {a.step.upper()}: L={left:+.0f} R={right:+.0f} for {a.seconds:g} s")
    print(f"Expected: {expect}")
    for i in (3, 2, 1):
        print(f"  starting in {i}...", flush=True)
        time.sleep(1)
    from motors import Motors
    m = Motors()
    try:
        end = time.monotonic() + a.seconds
        while time.monotonic() < end:
            m.drive(left, right)      # re-send faster than the motor watchdog
            time.sleep(0.05)
    finally:
        m.stop(block=True)            # active brake BRAKE_S, then coast
        m.cleanup()
    print("Done, motors stopped.")


if __name__ == "__main__":
    main()
