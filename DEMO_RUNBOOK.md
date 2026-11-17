# Demo runbook — Wed 2026-11-18

Car: Pi 5 + UFLD lane (INT8) + YOLO (NCNN) + ToF / ultrasonic safety net. Profile `ufld_yolo`.
Everything here was tested on the track on 2026-11-16/17 (see RESULTS_LOG.md, Part 8).

## 0. Night before / morning
- [ ] Charge the UPS (Pi) battery fully. All Pi reboots on 2026-11-16 happened at ≈ 9.8 V. Full = ≈ 12.3 V.
- [ ] Charge the motor battery (if separate). On 2026-11-17 the car ran wider on curves and felt slower.
- [ ] Bring: chargers, spare battery, the toys (person / car / animal), traffic light, stop sign, a cardboard box
      (unknown object), laptop/phone on the same Wi-Fi.
- [ ] Wheels clean (no hair / tape), track tape stuck down, no glare on the track.

## 1. Power on (≈ 2 min)
1. Car on a stand or held, **wheels off the ground**. Switch on.
2. Wait ≈ 2 min. SSH: `ssh veena_pi@veena-pi5` (or by IP: `hostname -I` on the Pi; it was `10.243.196.141`).
3. `cd ~/self_driving_car/pi_deploy`

## 2. Pre-flight + start the page
```bash
./preflight.sh          # all checks (camera, I2C sensors, models, FPS, temp, power), then starts the page
```
- All PASS → the page starts **STOPPED + DRY RUN**. Any FAIL → read the line; it asks before starting anyway.
- Fast restart without checks (e.g. after a crash):
  `~/car_project/venv/bin/python main_demo.py --web --profile ufld_yolo --phase P7`
- Only one program may hold the camera: stop `stream_camera.py` / other `main_demo.py` first (`pkill -f main_demo.py`).

Open **http://veena-pi5.local:8001** (or `http://<Pi IP>:8001`) on the laptop/phone. Press **Take control** if asked.

## 3. Settings to check on the page (do not change on the day)
| setting | value |
|---|---|
| base speed | 20 |
| Kp | 10 |
| Kd | 2 |
| Kh | 0 (5 only if tested and better) |
| turn limit | 20 |
| trims | 1.0 / 1.0 |
Fixed in config: red light stops at box height ≥ 0.10 anywhere in the image; stop sign ≥ 0.10 → stop 3 s, then ignore
signs 5 s; obstacle (person/car/animal) CAUTION 400–800 mm, STOP < 400 mm; safety net ToF < 200 mm (any object).

## 4. Running a phase
1. Tap the phase (changing phase always stops the car).
2. **DRY RUN check by hand first** (DRY RUN ticked, START, push the car): the page shows what it *would* do.
3. Car on the track, hands clear. **Untick DRY RUN**, press **START**. Hand near **E-STOP**.
4. After the run: **STOP**, then **PASS / FAIL** in the rehearsal table. **Tick DRY RUN again** before any hand check
   (on 2026-11-17 a "hand check" ran with motors live because DRY RUN had been left off).

| phase | what to show | expected |
|---|---|---|
| P1 | lane following | stays between the tapes for a lap |
| P2 | toy in the lane | CAUTION → STOP_OBSTACLE < 400 mm → toy removed → drives after 1 s |
| P3 | toy beside the lane | no stop |
| P4 | box in the lane | STOP_OBSTACLE "safety tof" < 200 mm → removed → drives |
| P5 | traffic light (left side ok) | red: CAUTION → STOP_RED at h 0.10; green → DRIVE |
| P6 | stop sign | STOP_SIGN 3 s → DRIVE |
| P7 | **full demo** | all of the above in one lap |

## 5. If something goes wrong
| symptom | do this |
|---|---|
| car heading off the track | **E-STOP** (any browser can press it), pick the car up |
| STOP_LANE_LOST | car can't see both tapes: put it back in the lane centre, press START |
| page not loading | Pi rebooted or page crashed: wait 2 min, SSH in, step 2 (`./preflight.sh`) |
| Pi reboots by itself | UPS battery low → charge / swap; check `vcgencmd get_throttled` (0x0 = ok) |
| "view only" | another browser has control: **Take control** (car must be stopped) |
| fail-safe on the page | a thread stalled / too hot / under-voltage / heartbeat lost: STOP, wait, START again |
| camera missing | `rpicam-hello --list-cameras`; re-seat the ribbon cable (CAM1), reboot |
| ToF -121 / sensor errors | I2C wiring intermittent: re-seat the ToF plug, re-run preflight |
| car wide on curves | battery low → charge; or Kh slider 5 (test in P1 first) |

Every exit path stops the motors; the page heartbeat stops the car if the browser goes away for 1.5 s.

## 6. After the demo
`git add logs RESULTS_LOG.md && git commit -m "demo day logs"`; shut down with `sudo poweroff` before switching off.
