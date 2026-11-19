# Self-driving car on a Raspberry Pi 5

A small 4-wheel car that follows a taped lane, stops for people, cars, animals and unknown
objects, obeys a red/green traffic light and a stop sign. Everything runs **on the car**
(Raspberry Pi 5, CPU only) and is controlled from a phone or laptop browser.

- **Lane following:** UFLD v2 lane-detection model, trained on my own track, INT8-quantised (≈ 10 FPS on the Pi)
- **Object detection:** YOLO26n fine-tuned on my own objects, exported to NCNN (≈ 10 FPS)
- **Safety net:** VL53L5CX time-of-flight + two ultrasonic sensors stop the car for *any* object < 200 mm
- **Control page:** E-STOP / START / STOP / DRY RUN, live camera with overlays, demo phases P1–P7

<!-- VIDEOS -->
## Demo

> Videos will be added here: full P7 demo (screen recording of the control page) and a phone video of the car on the track.

## How it works

```
                 ┌──────────────► UFLD v2 lane model (INT8, onnxruntime) ──► lane offset, heading ──┐
 camera thread ──┤                                                                                  │
 (picamera2)     └──────────────► YOLO26n (NCNN) ──► person / car / animal / light / stop sign ─────┤
                                                                                                    ▼
 sensor thread:  ToF (VL53L5CX), ultrasonics FL/FR, IMU ──► distance ───────────────►  decision.decide()
                                                                                       (pure function,
 web page :8001  E-STOP / START / STOP / DRY RUN / phase / heartbeat ──────────────►    unit-tested)
                                                                                                    ▼
 fail-safes: stale thread 1 s · lane FPS < 2.5 · CPU 80/85 °C · under-voltage · heartbeat 1.5 s ──► motors
                                                                                   (TB6612FNG, skid steer)
```

Camera, lane and detection each run in their own thread; a fixed-rate control loop (20 Hz) takes
the newest result of each, so a slow model never blocks steering or stopping. Every result
carries a timestamp and anything too old is ignored.

### Decision rules (highest priority first)

| state | when |
|---|---|
| `ESTOP` | E-STOP pressed (latched) |
| `STOP_OBSTACLE` (safety net) | ToF < 200 mm, or both ultrasonics < 200 mm — any object, no camera needed |
| `STOP_OBSTACLE` | person / car / animal in the lane corridor and ToF < 400 mm; resumes 1 s after it is gone |
| `STOP_RED` | red light seen for 3 checks with box height ≥ 0.10 of the frame; drives on green |
| `STOP_SIGN` | stop sign for 3 checks (height ≥ 0.10) → stop 3 s, then ignore signs for 5 s |
| `STOP_LANE_LOST` | no lane for 10 frames |
| `CAUTION` | obstacle 400–800 mm away, or a red light still far away → half speed |
| `DRIVE` | steer = Kp·offset + Kd·d(offset)/dt + Kh·heading, slower in sharp turns |

Full details: [`README_PI.md`](README_PI.md) §7 and [`decision.py`](decision.py).

### Demo phases

| phase | shows | expected |
|---|---|---|
| P1 | lane following | a full lap between the tapes |
| P2 | toy in the lane | CAUTION → STOP < 400 mm → resumes when removed |
| P3 | toy beside the lane | no stop |
| P4 | unknown box in the lane | safety-net stop < 200 mm |
| P5 | traffic light | stops on red, drives on green |
| P6 | stop sign | stops 3 s, then drives |
| P7 | **full demo** | all of the above in one lap |

## Results

All on the real track, logged in [`RESULTS_LOG.md`](RESULTS_LOG.md).

| test | result |
|---|---|
| UFLD point accuracy (578 track frames) | **93.7 %** (same as the training PC) |
| YOLO26n (Roboflow split 1255 / 359 / 179) | val mAP50 0.994, test 0.974, red ↔ green confusions **0** |
| Live speed, profile `ufld_yolo` | lane ≈ 10 FPS, detection ≈ 10 FPS, lane age < 0.4 s, 63–68 °C, no throttling |
| Soak test | 4 × 10 min (replay + live camera), 20 Hz control loop, memory flat, 0 exceptions, 0 fail-safe stops |
| P1 lane following (Kp 15) | 61 s lap, 0 lane-lost stops |
| P2–P6 | all PASS |
| P7 full demo (demo day rehearsal) | all rules OK; last run clean with 0 lane-lost stops |

## Hardware

| part | notes |
|---|---|
| Raspberry Pi 5 | Python 3.13, CPU only |
| Camera | 5 MP OV5647 on CAM1, 640×480 |
| Motor driver | TB6612FNG, one channel per side (4 DC motors, skid steer) |
| ToF | VL53L5CX 8×8 (I2C 0x29), centre-zone minimum |
| Ultrasonics | 2 × front-left / front-right |
| IMU | MPU6050 (I2C 0x68) |
| Power | UPS HAT with INA219 (I2C 0x41) for the Pi, separate motor battery |
| Cooling | fan on a relay (GPIO 18) |

Pins and every tunable value are in [`config.py`](config.py).

## Repository layout

| path | what |
|---|---|
| `main_demo.py` | entry point: `--web`, `--profile`, `--phase`, `--dry-run`, `--source` (replay) |
| `runtime.py`, `run_threaded.py` | camera / lane / detection threads and the fixed-rate control loop |
| `decision.py` | all driving rules (pure, unit-tested) |
| `failsafe.py` | stale threads, low lane FPS, temperature, under-voltage, heartbeat |
| `lanes/` | lane backends: `ufld.py` (main), `hough.py` (classic fallback), common `LaneResult` |
| `obstacle_yolo.py` | YOLO26n NCNN wrapper |
| `motors.py`, `sensors.py`, `camera.py` | hardware (lgpio, I2C sensors, picamera2) |
| `web.py` | control page on port 8001 (stdlib `http.server`, one controller, others view-only) |
| `models/` | NCNN YOLO model, UFLD row anchors + checksums (UFLD `.onnx` files: see Releases) |
| `test_*.py` | unit, parity and hardware tests, one per stage |
| `preflight.sh`, `soak_*.{sh,py}`, `bench_runtime.py` | pre-run checks, soak and benchmark tools |
| `deploy/car-demo.service` | systemd unit: control page starts on boot (STOPPED + DRY RUN) |
| `logs/` | selected run logs (CSV / JSONL) referenced from `RESULTS_LOG.md` |
| [`README_PI.md`](README_PI.md) | Pi setup and stage-by-stage testing guide |
| [`DEMO_RUNBOOK.md`](DEMO_RUNBOOK.md) | how the demo was run, troubleshooting table |

## Running it

```bash
# Pi 5, Raspberry Pi OS: apt install python3-picamera2 python3-lgpio i2c-tools
python3 -m venv --system-site-packages venv && . venv/bin/activate
pip install -r requirements_pi.txt
# put the UFLD .onnx files from the GitHub Release into models/ufld/ (checked against SHA256SUMS)

python test_decision.py && python test_control.py         # run anywhere, no hardware
./preflight.sh --check-only                               # camera, sensors, models, FPS, temp, power
python main_demo.py --web --profile ufld_yolo --phase P1  # open http://<pi>:8001
```

The page always starts **STOPPED + DRY RUN** (motors off; it shows what the car *would* do).
Untick DRY RUN and press START only with the car on the track and a hand near E-STOP.
Replay without hardware: `python main_demo.py --web --source <folder or video> --no-sensors` (replay always runs in dry run).

## Lessons learned

- **Check motor sides with the camera, not by eye.** The car kept drifting right; the left and right motor pins were swapped.
- **Power is a safety issue.** Every unexplained reboot was the UPS battery below ≈ 10 V.
- **Match training preprocessing exactly.** UFLD uses the same PIL resize as training; the parity test proves 0 px difference.
- **Gain depends on speed and battery.** Kp 10 under-steered on curves; Kp 15 fixed lane-lost stops.
- **Parity tests before driving.** Every model was checked on the Pi against the training PC before it touched the motors.
