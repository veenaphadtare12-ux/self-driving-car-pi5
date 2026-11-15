# Results log: Pi 5 lane following + obstacle stop

## Stage 1: Setup (2026-11-15) — PASS

**Environment**
- Raspberry Pi 5, Python 3.13.5, aarch64
- apt (already installed): python3-picamera2 0.3.39, python3-lgpio 0.2.2, python3-venv, i2c-tools 4.4. I2C enabled (`/dev/i2c-1`).
- venv: `venv/` with `include-system-site-packages = true`

**Deviation from README (approved by user):** `pip install -r requirements_pi.txt` could not
download torch 2.15.0 (484 MB; Wi-Fi ~12 KB/s, Ethernet dropped to ~25 KB/s at 47 %).
Instead the venv reuses torch from `~/ai_env` (same Python 3.13) through
`venv/lib/python3.13/site-packages/zz_ai_env.pth` (one line: `/home/veena_pi/ai_env/lib/python3.13/site-packages`).
Nothing in `~/ai_env` was copied or changed. pip then installed the rest of requirements_pi.txt into the venv.
If the venv is ever recreated, recreate this `.pth` file (venv/ is not in git).

| Package | Version | From |
|---|---|---|
| torch | 2.13.0+cpu | ~/ai_env |
| ultralytics | 8.4.157 (matches model export) | venv |
| ncnn | 1.0.20261020 | venv |
| numpy | 2.4.4 | ~/ai_env |
| opencv (cv2) | 5.0.0 | ~/ai_env |
| picamera2, lgpio | apt | system |

NumPy/picamera2 conflict: did not appear (`import picamera2` works with NumPy 2.4.4); no change made.

**Verify:** `python -c "import torch, ultralytics, lgpio, cv2, picamera2, ncnn, numpy"` → ok

**Unit tests**
- `test_decision.py`: Ran 59 tests, OK
- `test_control.py`: Ran 16 tests, OK (uses a fake lgpio; no real GPIO touched)

## Stage 2: Motors (2026-11-15) — PARTIAL (hardware fault found)

Wheels lifted. Runs at 30 % with the one-step runner (forward/backward/left 3–5 s each).
- Driver: TB6612, two channels (one per side), so both wheels on a side get the same signal.
- FORWARD and BACKWARD: right front, right back, left back follow the command.
  **Left front spins the opposite way** to left back.
- Conclusion: left front motor is wired with reversed polarity (soldered). Not fixable in
  config.py; `LEFT_INVERT`/`RIGHT_INVERT` stay False. Fix: re-solder the left front motor's
  two wires swapped, then re-run FORWARD/BACKWARD/LEFT/RIGHT.
- RIGHT step not yet run.

## Stage 2: Camera (2026-11-15) — FAIL (hardware)
`Picamera2()` → "No camera number 0 found"; `rpicam-hello --list-cameras` → "No cameras available!".
Not detected at boot (camera_auto_detect=1). Camera worked earlier in car_project → likely ribbon
cable loose/flipped. To do: power off, re-seat ribbon in CAM/DISP 0, re-run test_camera.py.
**Update 17:50:** re-seated cable + 2 reboots, still none. Forced driver (camera_auto_detect=0,
dtoverlay=ov5647 and ov5647,cam0; backup config.txt.bak_20261115_174404). Driver loads on both
ports but `ov5647 10-0036` and `11-0036` both fail: `i2c read error, reg: 300a = -121`, and
i2cdetect shows nothing at 0x36 on buses 10/11. The sensor does not answer on I2C at all → hardware:
ribbon (contacts facing wrong way, not fully in, or damaged), or the camera board itself.
Pi 5 needs the narrow 22-pin end at the Pi; a Pi 4-style 15-pin cable won't fit without an adapter.

## Stage 3: Distance sensors (2026-11-15) — PASS (one I2C dropout seen)
i2cdetect: 0x29 (VL53L5CX), 0x68 (MPU6050). ToF ranging at 15 Hz; both ultrasonics in alert mode.
Hand test (centre → left → right, ~40 cm in to ~4 cm and back): ToF, US-FL and US-FR all track
the hand smoothly (ToF 419 → 42 mm; US-FL/FR agree with ToF within ~10–40 mm when the hand
covers them). Ultrasonics show frequent "-" (no echo) on a hand, expected for a small soft target.
Gyro z at rest within ±0.5 rad/s.
**Issue:** one run started with 2449 × `OSError 121 Remote I/O` from the ToF and 0x29 vanished
from the bus, then the test hung on Ctrl+C (needed kill -9). Next two runs were clean (0 errors).
Suspect loose ToF wiring; recheck if it repeats. Run with `timeout -k 3 -s INT N` meanwhile.

## Offline checks on old car_project frames (2026-11-15) — while the camera is broken
40 frames from `~/car_project/debug/camera_check_frames*` (1280x720, packed into an mp4 via `--source`).
These are old photos, not a substitute for stages 4/5 on the live camera.
- **YOLO (stage 5 software):** NCNN model loads, ~26–30 ms/frame, **~36 FPS** at imgsz 416.
  Fans ON at start, OFF at exit. No detections (the frames contain no obstacles), as expected.
- **Lane (stage 4 software):** with the default white-tape HSV, no lane in 0/40. **The track tape is
  black.** Trial override HSV_LOWER=(0,0,0), HSV_UPPER=(179,255,80) on frames resized to 640x480:
  lane found 20/20 (hough), right line tracked well. V<=60 finds nothing; V<=100 also works.
  config.py NOT changed yet. Tune with tune_hsv.py on live frames once the camera works.

## Stage 3b: ToF calibration (2026-11-15) — PASS (3rd run)
Run 1: ToF + both US read a flat ~290–320 mm on most of the 8x8 grid in BOTH steps (empty / box at 400 mm);
only the top corners saw 2–3 m. Uniform distance = a surface ~30 cm ahead, not a floor gradient.
Run 2: **ToF init failed** (`OSError 121` during firmware upload; 0x29 back on the bus afterwards):
2nd ToF I2C dropout today → reseat/solder ToF wiring. US again constant (FL 313, FR 295) in both steps.
No config change. To do: confirm nothing within 30 cm / sensors not aimed down, fix ToF wiring, re-run.
Run 3 (after rewiring/clearing the front): ToF init OK.
- Empty track: ToF center zones all no-target/inf → OK. Close readings only in rows 6–7 (floor, ~400 mm)
  and corners, which are not used for decisions. US still see ~FL 470 / FR 438 mm on the empty track
  (floor echo or aimed slightly down). Harmless: decision.py uses US only for the safety net
  (BOTH < 200 mm). Optional: tilt the US up a little.
- Object at 400 mm: first samples ToF 418, US FL 414, FR 399 → within ±50. The object then moved
  (in to ~160 mm, out to ~330), so the summary medians/"error -110" are not meaningful; all three
  sensors tracked each other within ~10–30 mm through the sweep. No config change.
