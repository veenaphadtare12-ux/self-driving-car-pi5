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
