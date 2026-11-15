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

## Offline stage 6: main_demo dry-run on old frames (2026-11-15) — RUNS; steering too twitchy
`main_demo.py --dry-run --video old_640.mp4` (old car_project frames at 640x480, looped 3x = 120 frames),
live ToF/US, YOLO on. No motors (`--video` forces DryRunMotors). Log: logs/run_20261115_182513.csv.
- Whole pipeline runs end to end: lane found in 120/120 (black-tape HSV), YOLO 0 dets, ToF live,
  50–80 FPS (no camera wait), fans ON/OFF, clean exit.
- **Steering jitter:** lane offset steady (median 1.5 px change/frame, p90 4.2 px) but commands swing
  L=50/R=0 ↔ L=11/R=49; 30/120 frames steer opposite to the offset. Cause: STEER_KD=0.1 on px/s —
  2 px of jitter at 75 FPS = 150 px/s → 15 steer vs P term 0.2×22 ≈ 4.
  Replay of the same offsets at 30 FPS: KD 0.1 → 23 sign flips; KD 0.03 → 4; KD 0 → 0.
  Recommendation: STEER_KD ≈ 0.03 as a starting point, finalise in stage 7.

## Stage 2: camera (2026-11-15, after cable fix + reboot) — PASS (colour check pending)
`rpicam-hello --list-cameras` → ov5647 2592x1944 on CAM1 (i2c@88000, bus 10). CAM0 port still logs
`-121` (nothing plugged there — expected with the forced `dtoverlay=ov5647,cam0`).
`test_camera.py` → test_photo.jpg 640x480x3 uint8 OK. Photo is dim with a purple/green cast
(low light / possible NoIR module); CAM_FORMAT RGB888 → BGR in OpenCV per camera.py, believed correct.
To do: confirm colours on a known red/blue object; optionally drop the unused cam0 overlay.
Colour check (2026-11-15, live frame from `stream_camera.py`): orange wrist thread shows orange, blue
cloth blue, skin tones normal → R/B **not swapped**, CAM_FORMAT RGB888 is correct. Overall blue cast =
auto white balance in dim indoor light, not a channel bug. **Stage 2: PASS.**
`stream_camera.py` added: browser MJPEG preview on :8000 (centre lines; `--flip` for 180°).

## Stages 0, 4, 5, 5b, 6 on the track (2026-11-16) — camera fixed
Scene: car on the track before the crosswalk, toy car ahead in the lane, stop sign on the left.
- **Stage 0:** test_decision 59 OK, test_control 16 OK (tracebacks are injected faults).
- **Stage 4 lane — FAIL with old ROI, PASS after retune.** Camera is aimed far ahead (needed to see the
  signs), so the tape is only visible in rows ~80–330; old ROI 280–480 saw a sliver of the left line and
  gave offset −172 px with the car centred. Toy-car shadow and crosswalk stripes also matched the black mask.
  Changes: ROI 120–340, LOOKAHEAD_ROW 200, LANE_WIDTH_PX 450, HOUGH_MIN_LINE_LEN 80 (drops crosswalk
  stripes), new HOUGH_MAX_ABS_SLOPE 3.0 in lane_hough.py (drops sign pole / obstacle edges; lane lines ≈1).
  Live 900 frames: both lines every frame (hough), 0 lost, offset −2…−8 px centred.
  Not yet checked: sign of the offset when the car is pushed sideways; plain stretch/curves.
  Crosswalk stripes still enter the ROI as the car drives over them (filtered by length, watch in stage 7).
- **Stage 5 YOLO:** car conf 0.93 (in corridor), stop_sign conf ~0.68, ~35 FPS / 27 ms, fans ON→OFF.
- **Stage 5b signals:** stop_sign h 0.105–0.108, x 0.05 (left edge — leaves the frame if closer).
  STOP_SIGN_HEIGHT_RATIO 0.3 → **0.10**. No traffic light in view: RED_LIGHT_HEIGHT_RATIO / SIGNAL_REGION_X
  unchanged (lights expected on the right; this sign is on the left, but stop signs ignore SIGNAL_REGION_X).
- **Stage 6 dry-run (live, stationary):** 450 frames, STOP_OBSTACLE 448 / CAUTION 2, ToF 340 mm matches
  the toy car, commands 0/0, ~36 FPS, dry.mp4 + logs/run_20261116_001647.csv. Lane-following commands not
  yet exercised (obstacle in the way). STEER_KD still 0.1 (0.03 recommended, awaiting OK).

## UFLD integration — Part 0: safety tag and checks (2026-11-16 00:25)
- Tag **pre-ufld** → 9e2481d (Hough lane + YOLO, STEER_KD 0.03, live_ufld_yolo.py viewer). Return: `git checkout pre-ufld`.
- Power: `get_throttled=0x0` (no under-voltage / throttling, now or since boot); EXT5V 5.25 V; core 0.84 V.
- CPU 49.9 °C idle. RAM 4049 MB total, 2828 MB available, swap 0 used. Disk 81 GB free.
- Camera: ov5647 detected on CAM1 → Parts 1–3 can use the live camera (replay for repeatable tests).
- Note: stream_camera.py has an uncommitted 1-line change (except OSError) made outside this session; left as is.

## UFLD integration — Part 1: UFLD in pi_deploy (2026-11-16)
Copied (not symlinked) from ~/car_project into `models/ufld/` (ONNX files are git-ignored: 152 MB > GitHub's
100 MB limit; `models/ufld/SHA256SUMS` is committed and matches the car_project originals):
| file | SHA-256 |
|---|---|
| ufld_v2_same_mytrack.onnx | dbee946a71031c8ecef32f7e6ac5ad6f02227e939d6e9e8b8fe46ef385a7d050 |
| ufld_v2_plus_mytrack.onnx | f8c7ef7bbfde008d00c5aec094cc4fb81fea334285d25596daaeda91abb6fe39 |
| row_anchors.txt (export/lane_v2, 56 anchors 4…284) | be731a87ef37e23cb2361117999f3e6d50ad034b14682a2cc8cfd0e6f9235989 |

- onnxruntime 1.31.0 installed into the pi_deploy venv (same version as car_project); added to requirements_pi.txt.
- `lanes/ufld.py`: port of lane_infer.py + perception.py confidence. Preprocessing unchanged: invert → RGB →
  PIL bilinear 800x288 → ImageNet norm → model → softmax/expectation → 56 anchors → 2 lanes.
  **Port check: 10 s04 frames, same rows and max |dx| = 0.000000 px vs car_project lane_infer.py.**
- `lanes/result.py`: LaneResult (left/right points, centre, offset −1…+1 with + = lane centre RIGHT, heading,
  confidence, lines LR/L/R/none, t, source) + LaneEstimator ported from car/lane_estimator.py (look-ahead band
  320–420 of 720, single-line fallback with learned width, 3-frame smoothing, heading bias). Works in reference
  1280x720 coordinates, so 640x480 full-FOV frames give the same geometry.
- `lanes/hough.py`: wraps the unchanged lane_hough.LaneDetector; decision offset stays in pixels (as calibrated),
  LaneResult offset = px / (LANE_WIDTH_PX/2). lane_hough.py only gained `self.last_fits` (no behaviour change).
- `main_demo.py --lane {hough,ufld} [--ufld-model v2_same|v2_plus]`, default `config.LANE_BACKEND = "hough"`.
  Per-backend steering: Hough = STEER_KP/KD/MAX (unchanged); UFLD = STEER_KP_UFLD 10, STEER_KD_UFLD 2,
  STEER_MAX_UFLD 15 (car_project start values), passed to decide() as a config view — decision rules unchanged.
  CSV/HUD offset shows 3/2 decimals for UFLD (Hough format unchanged).
- Tests: **test_lanes.py 22/22 OK** (sign convention both ways, single L / single R fallback, width learning + limits,
  empty input, band/rows/confidence gates, smoothing, heading sign + bias, per-backend gains through decide(),
  Hough sign on synthetic tape images, UFLD colour order/invert, postprocess/anchors, real-model smoke);
  test_decision 59 OK, test_control 16 OK.
- Smoke (dry run, 100 s04 frames, no sensors): hough 100 rows OK (~81 FPS; old-mount frames give dropouts with the
  new ROI, expected); ufld 100 rows DRIVE, offsets 0.0–0.4, **4.1 FPS with UFLD + YOLO in the same loop** (Part 3 threads).
- Not done in Part 1: heading is computed/logged but decide() has no heading term (would be a rule change).
