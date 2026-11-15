# Pi 5 deployment: lane following + obstacle stop

```
camera (picamera2 640x480) ──► lane_hough  (every frame)  ──► offset_px ─┐
                           └─► obstacle_yolo (every 3rd)  ──► detections ┤
sensors thread: VL53L5CX center min, ultrasonics FL/FR ──► tof_mm ───────┤
                                                                          ▼
                                          decision.decide()  (pure, unit-tested)
                                                                          ▼
                                          motors.drive(left, right)  (lgpio, TB6612FNG)
```

| File | Purpose |
|---|---|
| `config.py` | Every tunable: pins, speeds, PD gains, HSV, ROI, YOLO, thresholds |
| `motors.py` | `Motors.drive/stop/cleanup` (lgpio); stop = brake 0.3 s then coast; SIGINT stop, 0.5 s watchdog; `DryRunMotors` |
| `gpio_chip.py` | Opens gpiochip0, falls back to gpiochip4 if the pins can't be claimed; prints which |
| `fans.py` | Fan relay (GPIO18, active-low): ON in `main_demo.py` / `test_yolo.py`, OFF on exit |
| `lane_hough.py` | HSV mask → ROI → Canny/Hough (centroid fallback) → offset + overlay |
| `obstacle_yolo.py` | NCNN YOLO26n (`models/obstacle_v1_ncnn_model`), imgsz 416 |
| `sensors.py` | ToF center min (15 Hz; `inf` = no target), alternating ultrasonics, gyro. Background thread with timestamps; never raises |
| `decision.py` | DRIVE / CAUTION / STOP_OBSTACLE / STOP_RED / STOP_SIGN / STOP_LANE_LOST / ESTOP (pure) |
| `main_demo.py` | The full loop (`control_loop()`), CSV with detection/sensor ages, exception → stop + log |
| `camera.py`, `keys.py` | Camera source (picamera2 or file), non-blocking terminal keys |
| `test_*.py`, `tune_hsv.py` | One test per stage, HSV tuner. `test_decision.py` + `test_control.py` run anywhere |
| `calibrate_tof.py` | Empty-track floor check + object-at-40 cm check for ToF and ultrasonics |
| `calibrate_signals.py` | Height, center x, region per light / stop sign; suggests thresholds |

## 1. Copy to the Pi

From the laptop (Git Bash):

```bash
cd ~/self_driving_car_training
ssh veena_pi@veena-pi5.local "mkdir -p ~/self_driving_car"
scp pi_deploy.zip veena_pi@veena-pi5.local:~/self_driving_car/
ssh veena_pi@veena-pi5.local "cd ~/self_driving_car && unzip -o pi_deploy.zip"
```

This gives `~/self_driving_car/pi_deploy/` on the Pi. The NCNN model is already inside
`pi_deploy/models/`. Re-running the unzip keeps an existing `venv/` and `logs/`.

## 2. One-time Pi setup

```bash
sudo raspi-config nonint do_i2c 0          # enable I2C (or: raspi-config > Interface Options > I2C)
sudo apt update
sudo apt install -y python3-picamera2 python3-lgpio python3-venv i2c-tools
sudo reboot
```

Check the I2C devices: you should see `29` (VL53L5CX) and `68` (MPU6050):

```bash
i2cdetect -y 1
```

## 3. venv (must use `--system-site-packages`)

picamera2 and lgpio come from apt. They are only visible in a venv created with
`--system-site-packages`:

```bash
cd ~/self_driving_car/pi_deploy
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -U pip
pip install -r requirements_pi.txt
python -c "import picamera2, lgpio, cv2, ultralytics, ncnn; print('ok')"
```

If `import picamera2` now fails with `numpy.dtype size changed`, pip installed a
NumPy that the apt packages weren't built for. Fix it with `pip install "numpy<2"`.

Do **not** `pip install RPi.GPIO` or `pigpio`. This code uses lgpio only.

## 4. Test one stage at a time (in this order)

| # | Command | Pass when |
|---|---|---|
| 0 | `python test_decision.py` and `python test_control.py` | 59 + 16 tests OK (no hardware needed) |
| 1 | `python test_motors.py` (**wheels off the ground**) | Prints `using gpiochipN`; FORWARD = all wheels forward; LEFT turns left; RIGHT turns right |
| 2 | `python test_camera.py` | `test_photo.jpg` looks right (colours not swapped) |
| 3 | `python test_i2c_sensors.py` | ToF and both ultrasonics change when you move your hand (`--grid` shows the 8x8) |
| 3b | `python calibrate_tof.py` | Empty track: no center zone sees the floor. Object at 40 cm: ToF and US read 400 ± 50 mm |
| 4 | `python test_lane.py` | Pushing the car by hand: offset ≈ 0 centered, + when the lane is to the right; check `lane_frames/` |
| 5 | `python test_yolo.py --save yolo_frames` | Fans switch on; detects your obstacles; note the FPS. Fans off on exit |
| 5b | `python calibrate_signals.py` (car at the stop line) | Set `RED_LIGHT_HEIGHT_RATIO`, `STOP_SIGN_HEIGHT_RATIO`, `SIGNAL_REGION_X` from its summary |
| 6 | `python main_demo.py --dry-run --save-video dry.mp4` | Push the car by hand; states and commands make sense |
| 7 | `python main_demo.py --save-video run1.mp4` | Real run at low speed |

Fixes when a stage fails:
* **A side spins backward:** flip `LEFT_INVERT` / `RIGHT_INVERT` in `config.py`.
* **Car drifts on a straight line:** lower `LEFT_TRIM` or `RIGHT_TRIM` (e.g. 0.93).
* **Car steers the wrong way:** usually the motor inversion above, not the lane sign. Check test 1 first.

## 5. Tuning the tape colour (on the laptop)

1. On the Pi, place the car on the track and run `python test_camera.py -o track.jpg`.
2. Copy it to the laptop: `scp veena_pi@veena-pi5.local:~/self_driving_car/pi_deploy/track.jpg .`
3. On the laptop: `python tune_hsv.py track.jpg`. Adjust until only the tape is white
   inside the cyan ROI box, then press `q`.
4. Paste the printed `HSV_LOWER` / `HSV_UPPER` into `config.py` and copy it back to the Pi.

Also set in `config.py`:
* `LANE_MODE`: `"two_lines"` (tape on both edges) or `"center_line"` (one tape to follow).
* `ROI_TOP` / `LOOKAHEAD_ROW`: higher = looks further ahead.
* `LANE_WIDTH_PX`: lane width in pixels at `LOOKAHEAD_ROW` (measure it in an overlay frame).
  It is used when only one edge is visible.

Test lane detection on photos on the laptop: `python test_lane.py --source track.jpg --out lane_out`.

## 6. Running

```bash
python main_demo.py                          # drive
python main_demo.py --dry-run                # no motors, prints each command
python main_demo.py --no-yolo                # lane + distance safety net only
python main_demo.py --save-video out.mp4     # overlay: lane, boxes, ToF, state, FPS
python main_demo.py --show                   # live window (desktop/VNC only)
```

Keys in the terminal (works over SSH): **q** quit · **space/e** E-STOP (latched) · **r** reset ·
**Ctrl+C** stop. In every exit path the motors stop (brake 0.3 s, then coast): the loop's exception
handler, the `finally` block, the SIGINT handler, atexit, and the 0.5 s watchdog if the loop hangs.
An exception in the loop is written to the CSV as state `EXCEPTION` and the program exits with code 1.

Each run writes `logs/run_YYYYmmdd_HHMMSS.csv` with: time, frame, offset, detections
(`cls:conf:height:center_x;...`), det_age, tof, tof_age, us_fl, us_fl_age, us_fr, us_fr_age,
sensors_alive, state, reason, left, right, fps. Ages are seconds since the reading was taken.
`tof = inf` means the ToF saw no target in range.

## 7. Decision rules (`decision.py`)

Freshness: YOLO detections older than `DET_MAX_AGE_S` (0.5 s) and sensor readings older than
`SENSOR_STALE_S` (0.3 s) are ignored (treated as None).

Priority from top to bottom:
1. **ESTOP**: key pressed. Latched until `r`.
2. **Safety net → STOP_OBSTACLE**: ToF < 200 mm, or **both** front ultrasonics < 200 mm, for
   2 consecutive readings. No YOLO needed.
3. **STOP_OBSTACLE**: a person/car/animal box **in the corridor** (center x within
   `CORRIDOR_X` = 0.25–0.75 of the width, box bottom in the lower 60 % of the frame) and
   * ToF < `STOP_DISTANCE_MM` (400), or
   * ToF has no reading and box height > 0.5 (vision fallback).

   Resume (also after a safety stop) only when the stop condition has been false for 1 s **and**
   ToF > 500 mm (`STOP_DISTANCE_MM + RESUME_MARGIN_MM`). If the ToF has no reading, the
   ultrasonics are used; with no distance sensor at all, only the 1 s applies.
4. **STOP_RED**: `light_red`, conf ≥ 0.5, center x in `SIGNAL_REGION_X` (0.4–1.0, right side),
   for 3 consecutive YOLO checks, with height ≥ `RED_LIGHT_HEIGHT_RATIO`. Ends after green is
   confirmed for 3 checks, or after 10 s with neither red nor green seen. Red + green in one
   frame counts as red.
5. **STOP_SIGN**: stop sign for 3 consecutive checks with height ≥ `STOP_SIGN_HEIGHT_RATIO`
   → stop 3 s, then ignore stop signs for 5 s.
6. **STOP_LANE_LOST**: no lane for `LANE_LOST_FRAMES`=10 frames.
7. **CAUTION** (both wheels × 0.5): corridor obstacle with ToF 400–800 mm or box height ≥ 0.3,
   or a confirmed red light that is still smaller than `RED_LIGHT_HEIGHT_RATIO` (far).
   CAUTION comes after STOP_LANE_LOST on purpose: a car that lost the lane stops.
8. **DRIVE**: `steer = KP*offset + KD*d(offset)/dt` (px, px/s);
   `left = BASE+steer`, `right = BASE-steer`, clamped to [`MIN_WHEEL_SPEED`, `MAX_SPEED`].

Notes:
* No stop sign in the training data is taller than 0.26 of the frame, and the dataset's red
  lights are 0.10–0.17. Run `calibrate_signals.py` at your stop line and set the thresholds.
* Objects outside the corridor are ignored, so the corridor must match your track. In the
  dataset photos many objects sit at the image edges (e.g. the test person at x = 0.09).

## 8. Tuning order on the track

1. `BASE_SPEED` low (20–25) first.
2. `STEER_KP` (0.2): steer = 0.2 × offset px, so 60 px → ±12 on the wheels. If the car zig-zags,
   lower KP. If it misses curves, raise it.
3. `STEER_KD`: add damping if it oscillates. Set it to 0 if steering is jittery.
4. `YOLO_EVERY_N`: lower it if FPS allows. Check FPS in the CSV.
5. Measure the real speed at `BASE_SPEED` (time it over 1 m). Stopping needs about
   speed × 0.35 s (camera + YOLO latency) + braking distance < `STOP_DISTANCE_MM` minus the sensor's
   offset behind the bumper. At 0.5 m/s that is ~175 mm + braking.
6. `STOP_DISTANCE_MM`, `CORRIDOR_X`, `CAUTION_*`: check the `reason`, `det_age` and `tof_age` columns.

## Troubleshooting

* **Which GPIO chip:** the scripts try gpiochip0, then gpiochip4 (older Pi 5 kernels), and print
  `using gpiochipN`. Change the order in `GPIO_CHIP_CANDIDATES` if needed.
* **"GPIO busy":** another script still owns the pins (e.g. robot_hardware.py / gpiozero in
  Thonny). Close it. Don't run two of these scripts at once.
* **ToF init is slow (~10 s):** normal, it uploads firmware. Check the log line
  `VL53L5CX ranging at 15 Hz`. If it warns instead, the ToF runs at 1 Hz and stops will be late.
  `inf` = no target in range, `-`/None = invalid reading.
* **`sensors_alive` = False in the CSV:** the sensor thread died or hung. Distance readings expire
  after 0.3 s, so only vision is active. Fix the sensor before driving.
* **Ultrasonic always `-`:** the echo pin needs a 5 V → 3.3 V divider. `test_i2c_sensors.py`
  shows `alert`/`poll` mode per sensor.
* **Fans (relay on GPIO18, active-low):** claimed HIGH (off), driven LOW to switch on, HIGH again
  on exit. After the script exits the pin is released. If the relay turns on when the pin floats,
  the fans may keep running until something drives GPIO18 high. Set `FAN_ACTIVE_LOW = False` if
  your relay turns out to be active-high.
* **Low FPS:** use `--no-yolo` to measure lane-only FPS. Don't use `--show` over SSH/X forwarding.
