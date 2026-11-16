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

## UFLD integration — Part 2: parity (2026-11-16)
**UFLD — PASS.** `test_parity_ufld.py` = car_project `benchmark_lane_model.accuracy()` metric copied unchanged,
detector = pi_deploy `lanes/ufld.py` (v2_same, 4 threads, PIL resize). Frames: s04 val (car_project export/lane_v2).
| set | point acc | mean |err| px | lane precision | lane recall |
|---|---|---|---|---|---|
| all 578 s04 val | **0.9373** | 6.36 | 0.915 | 0.962 |
| 150 evenly spaced | **0.9448** | 6.29 | 0.918 | 0.964 |
Reference (car_project PROGRESS.md, all 578): 0.9373 → identical. Threshold 0.930 met on both sets.
Point-by-point vs car_project `lane_infer.LaneDetector` on all 578 frames: 0 row-set mismatches, max |dx| 0.000000 px.
Pipeline 224 ms/frame median (4.5 fps, UFLD alone). Output: logs/parity/ufld_v2_same.{txt,json}.
(Note: there are 578 s04 val labels, not 150 — both reported.)

**YOLO red-never-green — PASS.** `test_parity_yolo.py`, NCNN through pi_deploy ObstacleDetector (imgsz 416),
scanned at conf 0.10, judged at YOLO_CONF 0.4. Frames: car_project export/obstacles_v1 train+valid, scene from file name.
| scene | frames | correct colour ≥0.4 | wrong colour ≥0.4 | max wrong-colour conf |
|---|---|---|---|---|
| s5_signal_red | 292 | 187 (64 %) | **0** light_green | 0.214 |
| s06_signal_green | 276 | 208 (75 %) | **0** light_red | 0.155 |
Caveat: these images were most likely in the Roboflow training export → checks export + wrapper, not generalisation.
Detection rate < 100 % includes frames where the light is far / out of view (not labelled per frame).

**YOLO NCNN vs Ultralytics PyTorch — NOT RUN.** The fine-tuned .pt (trained on Windows,
`self_driving_car_training/.../roboflow_export`, 2026-11-14) is not on the Pi (~/robotex_ai/best.pt is an unrelated
1-class "Flame" model). Ready to run: `python test_parity_yolo.py --ref path/to/best.pt` (box match IoU ≥ 0.5,
FAIL on any red↔green change or < 95 % of reference boxes matched).
Regression: test_decision 59 OK, test_control 16 OK, test_lanes 22 OK.

## UFLD integration — Part 3: threads + fixed-rate loop + benchmark (2026-11-16) — targets NOT met
**Code.** `runtime.py`: camera thread (newest frame + capture time), lane thread, detection thread (NCNN on every
Nth camera frame), fixed-rate control loop (CONTROL_HZ 20) in the main thread; `run_threaded.py`
(`main_demo.py --threaded / --source / --web`; without these flags main_demo runs exactly as before);
`camera.ReplayCamera` (folder/video at REPLAY_FPS, loops); `ObstacleDetector(threads=)` (NCNN threads must be set
before the model loads; also sets torch threads); `UFLD_ALLOW_SPINNING` (onnxruntime busy-wait off: speed only).
decide() unchanged. `LaneFeed` keeps decide()'s per-frame behaviour per LANE RESULT: new result → offset with
dt = capture interval; between results → "hold" (lane counter + derivative memory unchanged, last steering);
lane older than LANE_MAX_AGE_S 0.5 → counted lost every tick. A crashed thread stops the car.
Phases P1–P7 filter detections before decide() ("ignored by phase" in the CSV); safety net/lane-lost/E-STOP always on.
CSV = old 17 columns + lane_source, lane_fps, det_fps, lane_age, det_age_lat, cpu_temp, throttled, phase, running,
dry_run, decision, ignored, failsafe, tick_ms, lane_seq.
**Tests:** test_runtime.py 14/14 (incl. fixed-rate steering == old per-frame steering to 1e-9 on 60 irregular lane
frames, lane-lost at the same lane frame, stale lane → STOP_LANE_LOST ≤ 1.0 s); decision 59, control 16, lanes 22 OK.

**Benchmark matrix** (`bench_runtime.py`, 60 s each after 6 s warm-up, DRY RUN, sensors on, UFLD v2_same, spinning off).
Replay = replay/s04_640x480.avi at 30 fps; live = camera (30 fps). lane/det = results per second; ages = at the control
tick (what decide() sees). Control loop 20.00 Hz in all 36 runs; overruns 0 except one live run (1 overrun, 221 ms).
| UFLD thr | NCNN thr | N | replay lane | det | lane age mean/max | live lane | det | lane age mean/max | det age max (live) |
|---|---|---|---|---|---|---|---|---|---|
| 2 | 1 | 1/2/3 | 2.30/2.38/2.62 | 16.0/14.7/10.0 | 0.59–0.68 / 0.88–1.23 | 2.35/2.50/2.55 | 15.6/14.8/10.0 | 0.61–0.66 / 0.88–1.01 | 0.19–0.37 |
| 2 | 2 | 1/2/3 | 1.93/2.18/2.55 | 20.0/15.0/10.0 | 0.61–0.80 / 0.90–1.20 | 1.93/2.13/2.32 | 18.8/15.0/9.4 | 0.68–0.80 / 1.02–1.72 | 0.18–0.82 |
| 3 | 1 | 1/2/3 | 2.55/2.55/2.88 | 14.5/14.4/10.0 | 0.54–0.61 / 0.73–0.87 | 2.27/2.25/2.77 | 13.6/13.9/10.0 | 0.56–0.70 / 0.88–1.72 | 0.21–0.27 |
| 3 | 2 | 1/2/3 | 2.87/2.90/2.97 | 8.7/8.5/7.7 | 0.52–0.54 / 0.80–0.82 | 2.82/2.80/2.87 | 8.1/7.9/7.4 | 0.55–0.56 / 0.84–0.92 | 0.44–0.53 |
| 4 | 1 | 1/2/3 | 2.38/2.42/2.90 | 14.4/14.1/10.0 | 0.53–0.64 / 0.75–1.13 | 2.35/2.35/2.55 | 13.8/14.0/9.9 | 0.62–0.66 / 0.94–1.42 | 0.22–0.31 |
| 4 | 2 | 1/2/3 | 2.65/2.65/2.72 | 8.9/9.0/8.0 | 0.57–0.58 / 0.79–0.88 | 2.57/2.60/2.68 | 8.1/8.5/7.4 | 0.58–0.60 / 0.90–0.97 | 0.54–0.71 |
Max temperature 68.8 °C, throttled 0x0 throughout. **0/36 meet all targets** (lane ≥ 4 fps, det ≥ 5 fps, ages < 0.4 s).
Extra (YOLO every 6th frame = 5 fps): replay UFLD3/NCNN1 lane 3.67 det 5.00; **live UFLD4/NCNN1 lane 3.28, det 5.02,
lane age 0.48/0.70, det age 0.18/0.32, 20.00 Hz, 0 overruns, 67.8 °C**; live UFLD3/NCNN1 lane 3.20, det 5.02, lane age max 0.90.
Reference: UFLD alone in the runtime 4.20 (3 thr) / 4.53 (4 thr) fps; YOLO alone 47 ms (1 thr) / 31 ms (2) / 27 ms (4).
Spinning on vs off (replay, 20 s, N=3): UFLD3 2.90 vs 2.90, UFLD4 2.40 vs 2.80 → off.
Sensors on vs off (replay UFLD3/NCNN1/N3): lane 2.88 vs 3.07.
**Parallelism (GIL):** real. UFLD3+NCNN1 N=1 live: lane 2.27/4.20 + det 13.6/21.3 = 1.18 "alone-equivalents" per
second (> 1 means both models computed at the same time); onnxruntime and NCNN release the GIL. The limit is CPU:
UFLD wants ~3.6 core-seconds per lane frame at 4 fps on a 4-core Pi.
**Why the targets can't be met with this model:** UFLD alone = 4.5 fps → a result is ~0.22 s old on arrival and is used
until the next one, so the age at the control tick is ~0.22–0.45 s even with nothing else running; with YOLO ≥ 5 fps
the lane drops to ~3.3 fps (live) → max age ~0.7 s.
**Saved (best available, not meeting lane fps / lane age):** UFLD_THREADS 4, YOLO_THREADS 1, RT_YOLO_EVERY_N 6
(threaded runtime only; YOLO_EVERY_N 3 kept for the old frame loop), UFLD_ALLOW_SPINNING False.
Notes for decision: (1) at 3.3 lane fps LANE_LOST_FRAMES 10 = ~3 s of lost lane before STOP_LANE_LOST (Hough at
30 fps: 0.33 s) — not changed (calibrated threshold); (2) LANE_MAX_AGE_S 0.5 is below the 0.7 s max age → short
"stale" stretches count as lost frames; (3) INT8 UFLD measured below as the plan's fallback (proposal only).

### Extra check: red / green detection by box size (`test_red_light_sizes.py`)
obstacles_v1 photos, NCNN, sizes from the box (or ±5 neighbouring photos when missed). Buckets: small < 0.10,
medium 0.10–0.15, large ≥ 0.15 (= RED_LIGHT_HEIGHT_RATIO, the stopping size).
| scene | small found@0.4 | medium | large | size unknown |
|---|---|---|---|---|
| red (292) | 26/27 (96 %) | 152/189 (80 %) | **9/38 (24 %)** | 38 (no box, no neighbour) |
| green (276) | 30/30 (100 %) | 144/150 (96 %) | 34/67 (51 %) | 29 |
**The misses are the CLOSE lights, not far ones**: in every photo the light is on the LEFT of the frame (box centre
x 0.01–0.33); large lights sit at x 0.00–0.03, cut off by the frame edge. Confidence when found: median 0.8.
**3 consecutive checks (SIGNAL_CONFIRM_CHECKS):**
- with the current SIGNAL_REGION_X (0.4–1.0, right side): **0 % of frames pass → STOP_RED would never fire in this layout.**
- region ignored (light side assumed correct), at stopping size: per-check pass 0.24 → 3-in-a-row fails 92 % of
  3-photo windows (24/26; 99 % if independent). At medium size: per-check 0.80 → 26 % of windows fail (48 % iid).
Decision needed at Part 8.3 (not changed): put the light where it stands on the demo track, set SIGNAL_REGION_X to
that side, and pick RED_LIGHT_HEIGHT_RATIO at a size where the light is still fully in view (~0.12 here).
**YOLO NCNN vs best.pt:** still not run — ~/best.pt not present (checked 03:30).

### Part 3 addendum: INT8 UFLD — PROPOSAL (measured, NOT applied; config still uses the FP32 model)
`quantize_ufld.py` (onnx 1.23.2 in a separate venv ~/quant_env; pi_deploy venv unchanged). Candidates in
models/ufld/candidates/ (git-ignored). static_int8 SHA-256 d50f3deae53ea9f944b491d57851ce7d8beff8dc82bf74598fbe7b0fb1153e4a.
| model | size | model ms (4 thr, alone) | point acc all 578 | 150 subset | precision / recall |
|---|---|---|---|---|---|
| FP32 v2_same (deployed) | 76 MB | 197–203 | 0.9373 | 0.9448 | 0.915 / 0.962 |
| dynamic INT8 (all ops) | 38 MB | 145 | 0.9363 | 0.9432 | 0.924 / 0.958 |
| dynamic INT8 (FC head only) | 72 MB | 191 | not measured (no speed gain) | | |
| **static INT8 QDQ per-channel** (opset 11→13, 120 calibration frames from s01–s07, never s04) | 38 MB | **53** | **0.9405** | **0.9496** | 0.916 / 0.964 |
Both INT8 variants keep ≥ 92 % (plan's condition). **Static INT8 in the full threaded runtime, live camera, 30 s:**
| UFLD thr | NCNN thr | N | lane fps | det fps | lane age mean/max | det age max | Hz | temp | targets |
|---|---|---|---|---|---|---|---|---|---|
| 2 | 1 | 2 | 8.30 | 15.0 | 0.198 / 0.325 | 0.170 | 20.00 | 65.0 | MEETS ALL |
| 2 | 1 | 3 | 9.13 | 10.0 | 0.183 / 0.317 | 0.186 | 20.00 | 65.5 | MEETS ALL |
| 3 | 1 | 2 | 9.80 | 14.1 | 0.170 / 0.254 | 0.197 | 20.00 | 66.7 | MEETS ALL |
| 3 | 1 | 3 | **10.27** | 10.0 | 0.160 / **0.269** | 0.201 | 20.00 | 66.1 | MEETS ALL |
**Proposal:** deploy static INT8 v2_same with UFLD_THREADS 3, YOLO_THREADS 1, RT_YOLO_EVERY_N 3 (lane ~10 fps,
detection 10 fps, ages < 0.3 s). To apply (needs OK): copy to models/ufld/, add to UFLD_MODELS + SHA256SUMS, run
test_parity_ufld --path (done above) and a live lane check on the track (the 0.94 accuracy is on s04 frames).

## UFLD integration — Part 4: fail-safes (2026-11-16) — PASS
`failsafe.py` (checked every control tick; only stops or slows, decide() untouched; a STOP latches until START;
reason shown on the page and in the CSV `failsafe` column). Values in config (FAILSAFE_*).
`test_failsafe.py` 11/11 OK — each failure simulated on the threaded runtime (dry run, fake motors), measured:
| failure (simulated) | required | measured | result |
|---|---|---|---|
| lane thread crash (exception) | STOP | next tick ("lane thread crashed") | PASS |
| detection thread crash | STOP | next tick | PASS |
| lane output frozen | STOP ≤ 1 s | 1.001 s after the freeze | PASS |
| detection output frozen | STOP when 1 s without a result | 0.855 s after the freeze (last result was 0.15 s before it) | PASS |
| camera frozen | STOP ≤ 1 s | 1.051 s (camera + detection stale) | PASS |
| UFLD < 2.5 fps over 3 s while driving | STOP + offer Hough | 2.95 s after slowing to ~1.7 fps; Hough switch refused while driving, works stopped | PASS |
| CPU 82 °C | half speed | wheel speeds x 0.5 exactly, keeps driving | PASS |
| CPU 86 °C | STOP | 0.051 s (next tick) | PASS |
| under-voltage appears (0x50005) | log + warn | warning on page + CSV, keeps driving; old since-boot flag alone = no warning | PASS |
| page heartbeat lost while driving | STOP at 1.5 s | 1.552 s after the last heartbeat; regular heartbeats keep driving | PASS |
| STOP latch | stays stopped after recovery | stays STOPPED until START | PASS |
Note: with the FP32 setting (lane ~3.3 fps live) the UFLD-unhealthy margin is 0.8 fps; checked in the Part 6 soak.
Regression: decision 59, control 16, lanes 22, runtime 14 OK.

## UFLD integration — Part 5: demo page on port 8001 (2026-11-16) — PASS
`web.py` (stdlib http.server, daemon threads; design from car_project/car/web.py), `python main_demo.py --web
[--dry-run] [--source ...] [--lane ufld]`; prints the URL with the Pi's IP. Starts STOPPED + DRY RUN.
- Always-visible top bar: EMERGENCY STOP (works from ANY browser; turns DRY RUN on), START, STOP. DRY RUN switch
  (locked on with --source / --dry-run; real Motors are only created when switched off, stopped, after a confirm).
- One controlling browser (token + 0.5 s heartbeat); others view-only; takeover only while stopped or after 5 s
  without heartbeat. Heartbeat lost 1.5 s while driving → STOP (failsafe.py). Space/e E-STOP over SSH kept.
- Phases P1–P7 (highlighted; change stops the car and starts with fresh decision state, so no latch carries over);
  "ignored by phase: …" in the view + CSV. Lane selector Hough / UFLD v2_same / v2_plus (only stopped; loads in the
  background); "Switch to Hough" button when the UFLD fail-safe fires.
- Live view MJPEG 480x360 ~10 fps, drawn in its own thread only while watched/recording: lane points/lines + centre,
  YOLO boxes, corridor + signal region, state + reason, L/R, ToF/US, lane/det fps, control Hz, CPU temp, DRY RUN, REC;
  "CAMERA NOT CONNECTED" when there is no camera (car can't drive: camera fail-safe).
- RECORD: videos/<phase>_<time>.mp4 (stream rate) + logs/<phase>_<time>.csv. Status grid (lane/det fps, ages, CPU,
  throttled/UNDER-VOLTAGE), last 10 state changes, indicators camera/ToF/US L/US R/IMU.
- Part B: sliders base speed (cap WEB_BASE_SPEED_MAX 35) / Kp / Kd (of the active backend) / trims, applied live,
  Save → web_settings.json (only loaded by --web; config.py untouched); rehearsal tracker → rehearsal_log.csv
  (pass/fail/in-a-row, goal 5).
- SIGTERM/SIGINT now stop the threaded runtime cleanly even when started in the background (run_threaded.py).
- Fix found by the end-to-end check: after P7 saw an obstacle, switching to P1 kept STOP_OBSTACLE for ~1 s
  (decide()'s obstacle latch, OBSTACLE_CLEAR_S) → phase change now resets the decision state (car is stopped anyway).

**Tests.** test_web.py 10/10 (HTTP server + runtime with fakes): page loads, initial STOPPED/DRY RUN, buttons < 0.5 s
and effective, second browser view-only but E-STOP works, heartbeat loss → STOP ≤ 1.5 s, phase stop + ignored,
lane switch only stopped, RECORD mp4 + phase CSV, MJPEG frames, DRY RUN lock, settings cap/save, rehearsal streak,
no camera → banner + can't drive. test_runtime 15/15 (+ phase latch test).
**End-to-end** (`check_web_e2e.py` against `main_demo.py --web --source obstacles/s3 --lane ufld --no-sensors`,
real UFLD FP32 + YOLO, chosen setting):
| check | result |
|---|---|
| page load | 10.4 kB in 0.026 s |
| command response (phase/start/stop/estop) | max 5.7 ms (< 0.5 s) |
| control Hz / lane fps / det fps — page closed | 20.12 (min 20.0) / 3.22 / 5.01 |
| — stream open (view 9.8 fps) | 20.17 (min 20.0) / 3.17 / 5.02 |
| — stream + RECORD (view 9.9 fps) | 20.14 (min 20.0) / 3.08 / 4.98 → control rate unchanged (0 %), lane −4 % |
| P1 (camel in lane) | DRIVE all the time, "ignored by phase: animal" (after the fix) |
| P2 | STOP_OBSTACLE "animal h=0.68 (no tof)" / "clearing" hysteresis |
| P7 | STOP_OBSTACLE |
| second browser | claim 409, STOP 403, car keeps running; E-STOP 200 → ESTOP + DRY RUN |
| heartbeat (controller stops sending while driving) | STOP, "page heartbeat lost 1.6s" (1.5 s + tick after the last heartbeat) |
Frames: logs/web_e2e_replay/P1.jpg, P2.jpg, P7.jpg (overlays checked by eye: lane dots, boxes, regions, HUD).

## Reboot during the first soak test (investigated 2026-11-16 13:20) — cause: power cut
The 03:52 live soak (P7, sensors on, recording) stopped at 456 s (03:58); the Pi was next up at 12:55.
- Last CSV rows (logs/P7_20261116_035246.csv): 20 Hz, lane 3.33 fps, det 5.0, CPU 67.2 °C, `throttled` 0x0,
  no EXCEPTION/fail-safe row; file ends on a complete row. RSS flat ~645 MB of 4 GB. → not heat, not memory, not a software crash.
- After the reboot the RTC set the clock to **1970-01-01 00:00:15** (no RTC battery, BATT_V 0.003 V): the board had **no power at all**.
- systemd runs the hardware watchdog (1 min): a kernel hang would have rebooted by ~03:59, not 9 h later.
- `chosen/power/power_reset` = 0 (cold power-on); no previous-boot journal (journal is volatile, `journalctl -b -1` empty).
- Supply now: EXT5V 5.21–5.24 V, `throttled=0x0`; `max_current` = 0x384 (900 mA) and no USB-PD data → the Pi did not
  negotiate a 5 A USB-PD supply (powered from a non-PD source / 5V pins).
**Most likely:** the car's power source cut out (battery empty / BMS cut-off or connector), abrupt enough that no
under-voltage flag was recorded first. Fix: run long tests on the wall supply (27 W USB-C PD) or a full battery; check the
battery connector. Optional (not changed, needs OK): persistent journal (`/var/log/journal`) to keep logs across a cut.
Soak tests now log `get_throttled` every 10 s.

## INT8 UFLD applied for the ufld_yolo profile (2026-11-16) — approved
- models/ufld/ufld_v2_same_mytrack_int8.onnx = static INT8 candidate, SHA-256 d50f3deae53ea9f944b491d57851ce7d8beff8dc82bf74598fbe7b0fb1153e4a
  (in SHA256SUMS; preflight checks it). config: UFLD_MODELS["v2_same_int8"], RUNTIME_PROFILES, DEMO_PROFILE = ufld_yolo.
- `main_demo.py --profile {ufld_solo,ufld_yolo}` (explicit flags win); page lane selector + "UFLD v2_same_int8";
  `bench_runtime.py --profile` (+ `--ncnn 0` = no YOLO); preflight FPS test runs both profiles; preflight.sh starts DEMO_PROFILE.
  ufld_solo = FP32 v2_same, 4 threads, no YOLO; ufld_yolo = INT8, UFLD 3 threads, NCNN 1 thread, YOLO every 3rd frame.
- Accuracy of the installed file: test_parity_ufld --model v2_same_int8: **0.9405** all 578 (0.9496 subset), PASS.
- Replay benchmark (s04 video, 30 s, no sensors):
| profile | lane fps | det fps | ctrl Hz | overruns | lane age mean/max | det age max | temp | targets |
|---|---|---|---|---|---|---|---|---|
| ufld_solo (FP32, no YOLO) | 4.50 | – | 20.00 | 0 | 0.349 / **0.549** | – | 69.4 | lane age > 0.4 s |
| ufld_yolo (INT8 + YOLO) | **11.10** | 10.00 | 20.00 | 0 | 0.151 / 0.244 | 0.183 | 68.3 | MEETS ALL |
Note: ufld_solo is fast enough by FPS (≥ 4) but its lane age max misses the 0.4 s target (FP32 ~200 ms per frame).
Tests: profiles 6, decision 59, control 16, lanes 22, runtime 15, failsafe 11, web 10, stream_camera 2 — all OK.

## UFLD integration — Part 6: soak test (2026-11-16 13:35–14:18) — PASS
`soak_all.sh 10` (4 × 10 min, DRY RUN locked: replay = --source obstacles/s3 --no-sensors, live = --dry-run with sensors)
+ `soak_check.py` (controlling browser with heartbeat, MJPEG page view open, RECORD on, P7, START; /status every 5 s;
`get_throttled` + EXT5V + temperature every 10 s, fsync'd → logs/soak_<profile>_<mode>/power.csv).
| run | ctrl Hz mean/min | lane fps mean (min) | det fps | lane age at tick mean / p99 / max | det age max | temp per min (max) | RSS start → end | throttled (60 samples) | 5 V min–max | exceptions / fail-safe stops |
|---|---|---|---|---|---|---|---|---|---|---|
| ufld_solo replay | 20.14 / 20.0 | 3.87 (3.3) | – | 0.407 / 0.550 / **0.746** | – | 65.5–67.8 (68.3) | 322 → 324 MB (+0.6 %) | 0x0 | 5.10–5.22 | 0 / 0 |
| ufld_yolo replay | 20.18 / 20.0 | 9.98 (8.3) | 10.0 | 0.168 / 0.231 / 0.370 | 0.243 | 63.4–66.1 (67.8) | 530 → 535 MB (+0.1 %) | 0x0 | 5.09–5.20 | 0 / 0 |
| ufld_solo live | 20.15 / 20.0 | 4.03 (3.7) | – | 0.388 / 0.524 / **0.783** | – | 66.7–69.4 (70.0) | 345 → 349 MB (+0.8 %) | 0x0 | 5.07–5.20 | 0 / 0 |
| ufld_yolo live | 20.12 / 19.7 | 10.57 (9.3) | 10.0 | 0.161 / 0.228 / 0.347 | 0.306 | 65.5–68.3 (68.3) | 552 → 560 MB (+1.3 %) | 0x0 | 5.07–5.20 | 0 / 0 |
- CSV ticks = expected (20/s) in every run; recorded video 5935–5963 frames of ~6007 expected (98.8–99.3 %); page stream ~9.9 fps.
- One control overrun in total (ufld_yolo live, 12 s after START/RECORD began): one 179 ms period, tick work itself 0.16 ms
  (thread scheduled late during start-up), never again in 10 min; far below the 1 s stale fail-safe.
- States as expected: replay s3 = camel in lane → STOP_OBSTACLE (yolo) / DRIVE (solo, no YOLO); live on the desk = mostly
  STOP_LANE_LOST, safety net STOP_OBSTACLE when something was near the ToF.
- No power problem while on this supply: throttled 0x0 in all 240 samples, 5 V ≥ 5.07 V.
**Verdict:** both profiles stable for 10 min each, memory flat, 70 °C max. ufld_yolo (INT8) meets all targets;
ufld_solo (FP32) misses only the lane-age target (max 0.75–0.78 s, p99 0.52–0.55 s; fail-safe LANE_MAX_AGE_S 0.5 s →
occasional no-lane ticks are possible).

## UFLD integration — Part 7: pre-flight (2026-11-16 14:18) — 15 PASS, 0 WARN, 0 FAIL
`./preflight.sh --check-only` (without the flag it then starts `main_demo.py --web --profile ufld_yolo`, STOPPED + DRY RUN):
camera detected (ov5647) PASS · camera colours PASS (mean B,G,R 75/76/68) · ToF 450 mm PASS · ultrasonic L 1096 / R 202 mm PASS ·
IMU 0x70 PASS · model checksums PASS (incl. INT8) · models load PASS (v2_same, v2_plus, v2_same_int8, Hough, YOLO) ·
FPS ufld_solo PASS (lane 4.0, 20 Hz, lane age max 0.566 s) · FPS ufld_yolo PASS (lane 10.5, det 10.0, 20 Hz, age max 0.217 s) ·
CPU 60.6 °C PASS · throttled 0x0 PASS · disk 84.8 GB PASS · network http://192.168.0.130:8001/ PASS · fans PASS.
YOLO NCNN-vs-.pt parity: still pending (~/best.pt not on the Pi).

## ufld_solo switched to INT8 (2026-11-16 14:30) — approved
- config: `UFLD_SOLO_MODEL = "v2_same_int8"` (FP32 fallback: set it to "v2_same"; FP32 file kept, checksum + load still
  checked by preflight). Reason: FP32 solo soak lane age max 0.75–0.78 s > LANE_MAX_AGE_S 0.5; INT8 acc 0.9405 vs 0.9373.
- preflight FPS check now also needs lane age max < LANE_MAX_AGE_S (0.5 s) per profile.
- **5 min live soak, ufld_solo INT8** (DRY RUN, sensors on, page + stream open, RECORD, P7; logs/soak_ufld_solo_int8_live):
| ctrl Hz mean/min | period max | overruns | lane fps mean (min) | lane age at tick mean / p99 / max | temp per min (max) | RSS start → end | throttled (30 samples) | 5 V min | exceptions / stops |
|---|---|---|---|---|---|---|---|---|---|
| 20.15 / 20.0 | 57.5 ms | 0 | **13.94** (13.3) | 0.124 / 0.171 / **0.224** | 65.5–67.8 (68.8) | 234 → 236 MB (+0.4 %) | 0x0 | 5.07 V | 0 / 0 |
  CSV ticks 6005/6006, video 2982/3003 frames. Lane age max 0.224 s (< 0.5 s; FP32 was 0.783 s) → PASS.
  (The FP32 run's results stay in logs/soak_ufld_solo_live.)
Tests: profiles 6, decision 59, control 16, lanes 22, runtime 15, failsafe 11, web 10 — all OK.

## Pre-flight before the car tests (2026-11-16 14:43) — 15 PASS, 0 WARN, 0 FAIL
`./preflight.sh` (full: checks, then the page with --profile ufld_yolo, STOPPED + DRY RUN): FPS ufld_solo (INT8 4 t) lane 13.6,
age max 0.224 s; FPS ufld_yolo lane 10.2, det 10.0, age max 0.236 s; ToF 534 mm, US 1094/202 mm, 59.5 °C, throttled 0x0.

## Stage 1 motors re-run after the re-solder (2026-11-16) — PASS
Car on a stand, wheels lifted (user's phrase before each run), `motor_step.py` (new: one step per run, 3 s countdown,
then brake; speed ≤ 40, ≤ 3 s; motors.py unchanged), 20 % duty, 3 s each:
| step | command | user saw | result |
|---|---|---|---|
| forward | L +20 R +20 | all 4 wheels forward, same speed | PASS |
| backward | L −20 R −20 | all 4 wheels backward, same speed | PASS |
| left | L −10 R +20 | right side forward, left side slowly backward | PASS |
| right (first time) | L +20 R −10 | left side forward, right side slowly backward | PASS |
Left-front polarity fault fixed (2026-11-15: it spun opposite to left back). First attempt moved nothing: the motor
power switch was off. 20 % duty turns the wheels in the air (load on the track not yet known).
