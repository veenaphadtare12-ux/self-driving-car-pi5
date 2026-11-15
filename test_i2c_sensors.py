"""
Stage 3: distance sensors. Prints ToF (center-zone min), both front ultrasonics,
and gyro z every 0.5 s. Move your hand in front of the car to check them.

    python test_i2c_sensors.py
    python test_i2c_sensors.py --grid      # also print the full 8x8 ToF grid

'-' means None (no reading, out of range, or sensor missing).
If the ToF shows nothing: check `i2cdetect -y 1` for 0x29 (and 0x68 for the MPU6050).
"""
import argparse
import time

from sensors import Sensors


def fmt(v, spec=".0f"):
    return "-" if v is None else format(v, spec)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--grid", action="store_true")
    a = p.parse_args()

    s = Sensors()
    print(f"ToF ok={s.tof.ok if s.tof else False}  "
          f"US-FL ok={bool(s.us_fl and s.us_fl.ok)} ({getattr(s.us_fl, 'mode', '-')})  "
          f"US-FR ok={bool(s.us_fr and s.us_fr.ok)} ({getattr(s.us_fr, 'mode', '-')})  "
          f"gyro ok={s.gyro.ok if s.gyro else False}")
    try:
        while True:
            zones = None
            if a.grid and s.tof and s.tof.ok:
                end = time.monotonic() + 0.2
                while zones is None and time.monotonic() < end:
                    zones = s.tof.read_zones()
                    time.sleep(0.01)
                tof = s.tof.center_min_mm(zones)
            else:
                tof = s.read_tof_now()
            fl, fr = s.read_ultrasonic_now()
            gz = s.read_gyro_now()
            print(f"ToF center min {fmt(tof):>5} mm | US FL {fmt(fl):>5} mm  FR {fmt(fr):>5} mm | "
                  f"gyro z {fmt(gz, '+.3f')} rad/s")
            if zones is not None:
                dist = zones[0]
                for r in range(8):
                    print("   " + " ".join(f"{dist[r * 8 + c]:5d}" for c in range(8)))
            time.sleep(0.5)
    except KeyboardInterrupt:
        print()
    finally:
        s.close()


if __name__ == "__main__":
    main()
