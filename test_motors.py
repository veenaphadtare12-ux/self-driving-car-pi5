"""
Stage 1: motors. PUT THE CAR ON A BLOCK, WHEELS OFF THE GROUND.

    python test_motors.py            # forward, left, right, 2 s each at BASE_SPEED
    python test_motors.py --speed 40

Check: forward = all wheels spin forward; left = right side faster (or left
stopped/reversed); right = the opposite. If a side spins backward, flip
LEFT_INVERT / RIGHT_INVERT in config.py.
"""
import argparse
import time

import config
from motors import Motors


def hold(m, left, right, seconds, label):
    print(f"{label:8s} L={left:+.0f} R={right:+.0f} for {seconds:.0f}s")
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        m.drive(left, right)      # re-send faster than the watchdog timeout
        time.sleep(0.05)
    m.stop()
    time.sleep(0.7)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--speed", type=float, default=config.BASE_SPEED)
    p.add_argument("--seconds", type=float, default=2.0)
    a = p.parse_args()
    s = a.speed

    input("Wheels OFF the ground? Press Enter to start (Ctrl+C aborts)...")
    m = Motors()
    try:
        hold(m, s, s, a.seconds, "FORWARD")
        hold(m, -s * 0.5, s, a.seconds, "LEFT")
        hold(m, s, -s * 0.5, a.seconds, "RIGHT")
        print("Done.")
    except KeyboardInterrupt:
        print("\nAborted.")
    finally:
        m.stop()
        m.cleanup()


if __name__ == "__main__":
    main()
