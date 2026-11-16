"""
Every tunable for the Pi 5 car in one place.

No hardware imports here: decision.py and test_decision.py import this file
on the laptop.
"""
import os

_HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# GPIO (BCM numbers). lgpio only.
# ---------------------------------------------------------------------------
# Pi 5 with kernel >= 6.6.45: the 40-pin header is gpiochip0.
# Older Pi 5 kernels exposed it as gpiochip4. Chips are tried in this order;
# the first one where all the pins can be claimed is used (see gpio_chip.py).
GPIO_CHIP_CANDIDATES = (0, 4)

# TB6612FNG x2, skid-steer, STBY tied to VCC
# Pins from ~/car_project/config/motors.yaml (confirmed 2026-10-13, wheels lifted).
# LEFT/RIGHT = the car's real wheels seen from behind. IN1 = forward pin, IN2 = backward pin
# (motors.py: positive speed -> IN1 high, IN2 low).
LEFT_IN1, LEFT_IN2, LEFT_PWM = 23, 24, 13      # real left:  fwd 23, back 24, PWM 13
RIGHT_IN1, RIGHT_IN2, RIGHT_PWM = 22, 27, 12   # real right: fwd 22, back 27, PWM 12
PWM_FREQ_HZ = 1000

# No inversion: IN1/IN2 above are already forward/backward per side (motors.yaml invert_left/right: false).
# The old values (LEFT 27/22/12 inverted, RIGHT 23/24/13) drove forward correctly but had left/right mirrored.
LEFT_INVERT = False
RIGHT_INVERT = False
LEFT_TRIM = 1.0     # scale one side down if the car drifts on a straight line
RIGHT_TRIM = 1.0

# Cooling fans: relay on GPIO18, active-low (relay closes when the pin is LOW),
# as in robot_hardware.py. ON in main_demo.py and test_yolo.py, OFF on exit.
FAN_PIN = 18
FAN_ACTIVE_LOW = True

# Ultrasonic HC-SR04 (trig, echo). Echo pins need a 5V->3.3V divider.
US_FRONT_LEFT = (5, 6)
US_FRONT_RIGHT = (16, 26)
US_TIMEOUT_S = 0.025          # ~4 m round trip; bounds blocking time per read
# The two sensors are triggered alternately (one per sensor-loop cycle), never together.
# US_GAP_S = minimum time between any two pings, so the previous echo has died out.
US_GAP_S = 0.06
US_MIN_MM, US_MAX_MM = 20, 4000

# ---------------------------------------------------------------------------
# I2C sensors (bus 1)
# ---------------------------------------------------------------------------
I2C_BUS = 1
MPU6050_ADDR = 0x68
GYRO_Z_OFFSET = -0.58363      # rad/s drift measured in robot_hardware.py

# VL53L5CX 8x8 zones, row-major index = row*8 + col.
# "Center" = rows 2..5, cols 2..5 (a 4x4 block straight ahead).
TOF_RESOLUTION = 8 * 8
TOF_CENTER_ROWS = range(2, 6)
TOF_CENTER_COLS = range(2, 6)
TOF_MIN_VALID_MM = 20         # 0 and tiny values are "no target" in practice
TOF_MAX_VALID_MM = 4000
# target_status 5 = valid, 9 = valid with large pulse. Others are unreliable.
TOF_VALID_STATUSES = (5, 9)
# The VL53L5CX defaults to 1 Hz ranging if this is not set. 15 Hz is the 8x8 maximum.
TOF_RANGING_HZ = 15

SENSOR_POLL_HZ = 20           # background sensor thread rate
SENSOR_STALE_S = 0.3          # readings older than this are treated as None

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
CAM_WIDTH, CAM_HEIGHT = 640, 480
# picamera2 "RGB888" is stored B,G,R in memory, which is what OpenCV and
# ultralytics expect. Do not convert colors after capture.
CAM_FORMAT = "RGB888"

# ---------------------------------------------------------------------------
# Speeds (percent duty, -100..100)
# ---------------------------------------------------------------------------
BASE_SPEED = 30
MAX_SPEED = 50
MIN_WHEEL_SPEED = 0           # lane following never reverses a wheel; set <0 to allow pivoting
MOTOR_WATCHDOG_S = 0.5        # motors stop if drive() is not called for this long
BRAKE_S = 0.3                 # stop(): active brake (IN1=IN2=1, PWM 100) this long, then coast

# ---------------------------------------------------------------------------
# Steering (PD on lane offset in pixels; derivative in px/s)
# ---------------------------------------------------------------------------
STEER_KP = 0.2
STEER_KD = 0.03
STEER_MAX = MAX_SPEED         # clamp on the steering term

# ---------------------------------------------------------------------------
# Lane detection (tape)
# ---------------------------------------------------------------------------
# PLACEHOLDERS: tune with tune_hsv.py on a photo from the car's camera.
# OpenCV HSV ranges: H 0..179, S 0..255, V 0..255. Track uses BLACK tape.
# Starting point from old car_project frames (V<=80 found 20/20); retune with tune_hsv.py on live camera.
# White tape would be (0, 0, 180) .. (179, 60, 255).
HSV_LOWER = (0, 0, 0)
HSV_UPPER = (179, 255, 80)

# "two_lines": tape on both lane edges. Center = midpoint of left+right lines.
# "center_line": a single tape line to follow. Center = the line itself.
LANE_MODE = "two_lines"

# Rows of the 480-row image that are searched (bottom part, nearest the car)
ROI_TOP = 120               # tape is only visible above ~row 330 (camera aimed far to see signs)
ROI_BOTTOM = 340
LOOKAHEAD_ROW = 200           # row where the lane center is measured
LANE_WIDTH_PX = 450           # lane width at LOOKAHEAD_ROW, used when only one line is seen

MORPH_KERNEL = 5
CANNY_LOW, CANNY_HIGH = 50, 150
HOUGH_RHO = 1
HOUGH_THRESHOLD = 30
HOUGH_MIN_LINE_LEN = 80       # drops short crosswalk stripes
HOUGH_MAX_LINE_GAP = 20
HOUGH_MIN_ABS_SLOPE = 0.3     # ignore near-horizontal segments
HOUGH_MAX_ABS_SLOPE = 3.0     # ignore near-vertical segments (sign poles, obstacle edges); None = off
MIN_MASK_PIXELS = 300         # less than this in the ROI = no lane

# ---------------------------------------------------------------------------
# Lane backend: "hough" (above, tape mask + Hough) or "ufld" (trained UFLD v2 model, lanes/ufld.py).
# main_demo.py --lane overrides. Each backend has its own steering gains (offsets are in different units).
# ---------------------------------------------------------------------------
LANE_BACKEND = "hough"
LANE_SMOOTH_FRAMES = 3        # UFLD: average offset/heading over the last N valid frames
LANE_HEADING_BIAS = 0.0       # radians subtracted from the heading (camera/mount yaw; car straight + centred)

UFLD_MODELS = {
    "v2_same": os.path.join(_HERE, "models", "ufld", "ufld_v2_same_mytrack.onnx"),
    "v2_plus": os.path.join(_HERE, "models", "ufld", "ufld_v2_plus_mytrack.onnx"),
    # static INT8 QDQ of v2_same (Part 3 addendum): acc 0.9405 on all 578 s04 frames (FP32 0.9373), 53 ms vs ~200 ms
    "v2_same_int8": os.path.join(_HERE, "models", "ufld", "ufld_v2_same_mytrack_int8.onnx"),
}
UFLD_MODEL = "v2_same"
UFLD_ROW_ANCHORS = os.path.join(_HERE, "models", "ufld", "row_anchors.txt")   # 56 anchors, data-v2 package
UFLD_SHA256 = os.path.join(_HERE, "models", "ufld", "SHA256SUMS")
UFLD_THREADS = 4              # onnxruntime intra-op threads (Part 3 benchmark decides the final value)
UFLD_RESIZE = "pil"           # exactly like training; do not change
UFLD_ALLOW_SPINNING = False   # onnxruntime idle threads busy-wait if True (its default); False frees CPU for YOLO
                              # (Part 3: speed only, outputs identical; slightly better with YOLO running)
# Estimator, in REFERENCE coordinates 1280x720 (the training frames); from car_project lane_follow.yaml
UFLD_IMAGE_CENTER_X = 640
UFLD_LOOKAHEAD_ROWS = (320, 420)   # offset band, ~42-51 cm ahead of the camera
UFLD_HEADING_ROWS = (300, 480)
UFLD_WIDTH_PX_BY_ROW = {300: 994, 320: 1029, 340: 1062, 360: 1096, 380: 1124, 400: 1147,
                        420: 1172, 440: 1194, 460: 1214, 480: 1220}   # median two-line width, data-v2 labels
UFLD_WIDTH_LEARN_RATE = 0.05
UFLD_WIDTH_SCALE_LIMITS = (0.8, 1.2)
UFLD_MIN_ROWS_IN_BAND = 3
UFLD_MIN_CONFIDENCE = 0.45
UFLD_SINGLE_LINE_CONFIDENCE = 0.8
# Steering for UFLD: offset is normalised (-1..+1 = half a lane width), so gains are per unit offset.
# Start values = car_project lane_follow.yaml (kp 10, kd 2, turn_limit 15); tune on the track (Part 8).
STEER_KP_UFLD = 10.0
STEER_KD_UFLD = 2.0
STEER_MAX_UFLD = 15.0
# (Hough uses STEER_KP / STEER_KD / STEER_MAX above, offset in pixels.)

# ---------------------------------------------------------------------------
# YOLO
# ---------------------------------------------------------------------------
YOLO_MODEL_PATH = os.path.join(_HERE, "models", "obstacle_v1_ncnn_model")
YOLO_IMGSZ = 416
YOLO_CONF = 0.4
YOLO_EVERY_N = 3

# Classes in data.yaml: animal, car, light_green, light_red, person, stop_sign
ALL_CLASSES = ["animal", "car", "light_green", "light_red", "person", "stop_sign"]
OBSTACLE_CLASSES = ["person", "car", "animal"]   # things in the road; traffic classes have own rules
TRAFFIC_CLASSES = ["light_red", "light_green", "stop_sign"]

# ---------------------------------------------------------------------------
# Decision: freshness
# ---------------------------------------------------------------------------
DET_MAX_AGE_S = 0.5           # YOLO detections older than this are ignored
# (sensor readings expire after SENSOR_STALE_S, above)

# ---------------------------------------------------------------------------
# Decision: obstacles
# ---------------------------------------------------------------------------
# Safety net, no YOLO needed: ToF < SAFETY_STOP_MM, or BOTH front ultrasonics < SAFETY_STOP_MM,
# for SAFETY_CONSECUTIVE consecutive sensor readings.
SAFETY_STOP_MM = 200
SAFETY_CONSECUTIVE = 2

# Driving corridor: an obstacle box counts only if its center x is inside CORRIDOR_X
# (fraction of frame width) and its bottom edge is in the lower 60 % of the frame.
CORRIDOR_X = (0.25, 0.75)
CORRIDOR_BOTTOM_MIN = 0.4     # box bottom y / frame height must be >= this

STOP_DISTANCE_MM = 400        # corridor obstacle + ToF below this -> STOP_OBSTACLE
CAUTION_DISTANCE_MM = 800     # corridor obstacle + ToF in [STOP, CAUTION) -> CAUTION
STOP_BOX_HEIGHT_RATIO = 0.5   # ToF missing: corridor box taller than this -> STOP_OBSTACLE
CAUTION_BOX_HEIGHT_RATIO = 0.3  # corridor box at least this tall -> CAUTION
CAUTION_FACTOR = 0.5          # CAUTION multiplies both wheel speeds by this

# Resume after an obstacle stop only when BOTH hold:
OBSTACLE_CLEAR_S = 1.0        #   stop condition false for this long
RESUME_MARGIN_MM = 100        #   and ToF > STOP_DISTANCE_MM + RESUME_MARGIN_MM
# If the ToF has no reading at all, the front ultrasonics are used for the distance check;
# with no distance sensor at all (e.g. --no-sensors) only the clear time applies.

LANE_LOST_FRAMES = 10

# ---------------------------------------------------------------------------
# Decision: traffic lights (calibrate with calibrate_signals.py)
# ---------------------------------------------------------------------------
RED_LIGHT_CLASS = "light_red"
GREEN_LIGHT_CLASS = "light_green"
SIGNAL_CONF = 0.5
SIGNAL_REGION_X = (0.4, 1.0)  # lights stand on the right side: box center x in this range
SIGNAL_CONFIRM_CHECKS = 3     # consecutive YOLO checks needed to confirm red / green
# PLACEHOLDER: red box at least this tall = close enough to stop (STOP_RED); smaller = CAUTION.
# The dataset's red lights are 0.10-0.17 tall. Measure yours at the stop line.
RED_LIGHT_HEIGHT_RATIO = 0.15
RED_TIMEOUT_S = 10.0          # STOP_RED ends after this long with neither red nor green seen

# ---------------------------------------------------------------------------
# Decision: stop sign
# ---------------------------------------------------------------------------
# Confirmed for STOP_SIGN_CONFIRM_CHECKS consecutive YOLO checks -> stop STOP_SIGN_WAIT_S,
# then drive on and ignore stop signs for STOP_SIGN_IGNORE_S.
STOP_SIGN_CLASS = "stop_sign"
STOP_SIGN_CONF = 0.5
STOP_SIGN_HEIGHT_RATIO = 0.10 # measured 0.106 at the stop line 2026-11-16; sign leaves the frame if closer
STOP_SIGN_CONFIRM_CHECKS = 3
STOP_SIGN_WAIT_S = 3.0
STOP_SIGN_IGNORE_S = 5.0

# ---------------------------------------------------------------------------
# Threaded runtime (runtime.py; main_demo.py --threaded / --web). Part 3 benchmark picks the values.
# ---------------------------------------------------------------------------
CONTROL_HZ = 20               # fixed-rate control loop
LANE_MAX_AGE_S = 0.5          # lane result older than this = no lane
# Part 3 benchmark (live, 60 s): best available = UFLD 4 threads + NCNN 1 thread + YOLO every 6th frame:
# lane 3.28 fps, detection 5.02 fps, 20.00 Hz, lane age max 0.70 s, det age max 0.32 s, 67.8 C.
# No setting reaches lane >= 4 fps / lane age < 0.4 s with the FP32 UFLD (see RESULTS_LOG Part 3).
YOLO_THREADS = 1              # NCNN CPU threads in the threaded runtime (None = all cores)
RT_YOLO_EVERY_N = 6           # threaded runtime: YOLO on every Nth camera frame (30 fps camera -> 5 fps)
                              # (YOLO_EVERY_N above stays the frame-loop value for the old main_demo)
REPLAY_FPS = 30               # --source: frames per second delivered like a camera
# Runtime profiles: main_demo.py --profile NAME sets lane backend, UFLD model, threads and YOLO on/off
# (explicit --lane/--ufld-model/--ufld-threads/--yolo-threads/--yolo-every/--no-yolo still win).
#   ufld_solo  UFLD alone (no YOLO). INT8 since 2026-11-16: the FP32 soak gave lane 3.9-4.0 fps, lane age max
#              0.75-0.78 s (> LANE_MAX_AGE_S 0.5); INT8 is faster and more accurate (0.9405 vs 0.9373, 578 frames)
#   ufld_yolo  UFLD static INT8 + YOLO (approved 2026-11-16): soak live 10 min: lane 10.6 fps, det 10 fps,
#              lane age max 0.35 s, 20 Hz, 68 C (FP32 + YOLO only reached lane 3.3 fps)
UFLD_SOLO_MODEL = "v2_same_int8"   # FP32 fallback: "v2_same" (file kept in models/ufld/, checksum still checked)
RUNTIME_PROFILES = {
    "ufld_solo": {"lane": "ufld", "ufld_model": UFLD_SOLO_MODEL, "yolo": False, "ufld_threads": 4},
    "ufld_yolo": {"lane": "ufld", "ufld_model": "v2_same_int8", "yolo": True, "ufld_threads": 3,
                  "yolo_threads": 1, "yolo_every": 3},
}
DEMO_PROFILE = "ufld_yolo"    # preflight.sh starts the demo page with this profile
# Fail-safes (failsafe.py, Part 4). They only stop or slow the car; a stop latches until START.
FAILSAFE_STALE_S = 1.0        # lane / detection / camera thread published nothing for this long -> STOP
FAILSAFE_UFLD_MIN_FPS = 2.5   # UFLD lane FPS below this ...
FAILSAFE_UFLD_WINDOW_S = 3.0  # ... averaged over this window, while driving -> STOP (+ offer Hough)
FAILSAFE_TEMP_SLOW_C = 80.0   # CPU at/above -> half speed
FAILSAFE_TEMP_STOP_C = 85.0   # CPU at/above -> STOP
FAILSAFE_HOT_FACTOR = 0.5     # CPU above FAILSAFE_TEMP_SLOW_C -> both wheel speeds x this
FAILSAFE_HEARTBEAT_S = 1.5    # demo page heartbeat missing this long while driving -> STOP
WEB_BASE_SPEED_MAX = 35       # demo page base-speed slider cap (%)
PREFLIGHT_LANE_MARGIN_FPS = 0.3   # preflight FPS test: lane >= FAILSAFE_UFLD_MIN_FPS + this
PREFLIGHT_MIN_DET_FPS = 4.5       # preflight FPS test: detection >= this

# ---------------------------------------------------------------------------
# Logging / video
# ---------------------------------------------------------------------------
LOG_DIR = os.path.join(_HERE, "logs")
VIDEO_FPS = 15
