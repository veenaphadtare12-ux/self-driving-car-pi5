# Demo runbook — Wed 2026-11-18

Car: Pi 5 + UFLD lane (INT8) + YOLO (NCNN) + ToF / ultrasonic safety net. Profile `ufld_yolo`.
Everything here was tested on the track on 2026-11-16/17 (see RESULTS_LOG.md, Part 8).

## 0. Night before / morning
- [ ] Charge the UPS (Pi) battery fully. All Pi reboots on 2026-11-16 happened at ≈ 9.8 V. Full = ≈ 12.3 V.
- [ ] Charge the motor battery (if separate). On 2026-11-17 the car ran wider on curves and felt slower.
- [ ] Bring: chargers, spare battery, the toys (person / car / animal), traffic light, stop sign, a cardboard box
      (unknown object), laptop/phone on the same Wi-Fi.
- [ ] Wheels clean (no hair / tape), track tape stuck down, no glare on the track.

## 1. Power on — no SSH needed (≈ 2 min)
The page starts by itself on boot (systemd service `car-demo`, `deploy/car-demo.service`), always
**STOPPED + DRY RUN**, phase P1. If it crashes it restarts by itself after 5 s (again STOPPED + DRY RUN).
1. Turn on the **Realme phone hotspot** first (the Pi joins it automatically; "Rudra" Wi-Fi also works).
   Connect the laptop to the same hotspot.
2. Car on a stand or held, **wheels off the ground**. Switch on.
3. Wait ≈ 2 min. Open **http://veena-pi5.local:8001** on the laptop.
   If that does not load (phones often can't use `.local`): phone → Hotspot → Connected devices → `veena-pi5` →
   its IP → `http://<that IP>:8001` (it was `http://10.243.196.141:8001`).
4. Press **Take control** if asked. Check the camera picture moves and the lane dots sit on the tapes.

## 2. Optional: pre-flight checks over SSH
```bash
ssh veena_pi@veena-pi5.local
cd ~/self_driving_car/pi_deploy
sudo systemctl stop car-demo          # the checks need the camera
./preflight.sh --check-only           # camera, I2C sensors, models, FPS, temp, power
sudo systemctl start car-demo         # page back (≈ 30 s)
```
- Restart a stuck page: `sudo systemctl restart car-demo`. Page log: `logs/boot/server.txt`.
- Turn autostart off after the demo: `sudo systemctl disable --now car-demo`.
- Only one program may hold the camera: never run `stream_camera.py` / `test_camera.py` while the service runs.

## 3. Settings to check on the page (do not change on the day)
| setting | value |
|---|---|
| base speed | 20 |
| Kp | 15 (2026-11-18: 10 under-steered on curves) |
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
| page not loading | Pi rebooted or page crashed: wait 2 min and reload (it starts by itself). Still nothing: check the laptop is on the same hotspot, try the IP instead of `.local` |
| car stops by itself, "heartbeat" | the browser went away for 1.5 s (phone screen locked / tab switched): keep the screen on, press START |
| Pi reboots by itself | UPS battery low → charge / swap; check `vcgencmd get_throttled` (0x0 = ok) |
| "view only" | another browser has control: **Take control** (car must be stopped) |
| fail-safe on the page | a thread stalled / too hot / under-voltage / heartbeat lost: STOP, wait, START again |
| camera missing | `rpicam-hello --list-cameras`; re-seat the ribbon cable (CAM1), reboot |
| ToF -121 / sensor errors | I2C wiring intermittent: re-seat the ToF plug, re-run preflight |
| car wide on curves | battery low → charge; or Kh slider 5 (test in P1 first) |

Every exit path stops the motors; the page heartbeat stops the car if the browser goes away for 1.5 s.

## 6. After the demo
Press **STOP**. Best: SSH in and `sudo poweroff`, wait for the green LED to stop blinking, then switch off.
Without SSH: press STOP and switch off (the car is idle, so the risk is small).
Later: `git add logs RESULTS_LOG.md && git commit -m "demo day logs"`.
