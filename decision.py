"""
Pure decision logic. No hardware, no clock, no I/O, so it is unit-testable on the laptop.

    st = DecisionState()
    out, st = decide(DecisionInputs(t=now, offset_px=..., detections=dets, det_t=..., tof_mm=..., tof_t=...,
                                    us_fl_mm=..., us_fl_t=..., us_fr_mm=..., us_fr_t=...), st)
    motors.drive(out.left, out.right)

Inputs carry the time each reading was taken, and decide() applies the age limits itself:
  * detections older than DET_MAX_AGE_S are ignored
  * sensor readings older than SENSOR_STALE_S are treated as None
  * tof_mm == math.inf means a fresh ToF frame with no target in range (clear), while
    None means no valid reading (unknown)
  * a *_t of None with a value present means "measured now" (convenient in tests)

Priority (highest first):
  ESTOP           estop flag (main_demo latches it: space/e, reset with r)
  STOP_OBSTACLE   safety net: ToF < SAFETY_STOP_MM, or both front ultrasonics < SAFETY_STOP_MM,
                  for SAFETY_CONSECUTIVE consecutive readings (no YOLO needed)
  STOP_OBSTACLE   corridor obstacle (person/car/animal, center x in CORRIDOR_X, bottom in the lower
                  60 %) and ToF < STOP_DISTANCE_MM; or, with no ToF reading, box height > STOP_BOX_HEIGHT_RATIO.
                  Both kinds resume only after the stop condition has been false for OBSTACLE_CLEAR_S
                  AND ToF > STOP_DISTANCE_MM + RESUME_MARGIN_MM.
  STOP_RED        red light (right side, conf >= SIGNAL_CONF) confirmed for SIGNAL_CONFIRM_CHECKS YOLO
                  checks with height >= RED_LIGHT_HEIGHT_RATIO. Ends when green is confirmed for
                  SIGNAL_CONFIRM_CHECKS checks, or after RED_TIMEOUT_S with neither red nor green seen.
                  Red and green in the same frame count as red.
  STOP_SIGN       stop sign confirmed for STOP_SIGN_CONFIRM_CHECKS checks, height >= STOP_SIGN_HEIGHT_RATIO:
                  stop STOP_SIGN_WAIT_S, then ignore stop signs for STOP_SIGN_IGNORE_S.
  STOP_LANE_LOST  no lane for LANE_LOST_FRAMES frames
  CAUTION         corridor obstacle at STOP..CAUTION_DISTANCE_MM or box height >= CAUTION_BOX_HEIGHT_RATIO,
                  or a confirmed red light that is still small (far): lane following at CAUTION_FACTOR speed
  DRIVE           PD steering on the lane offset + STEER_KH x lane heading, base speed cut by up to
                  STEER_SLOWDOWN at full turn (short dropouts keep the last steering)

CAUTION is checked after STOP_LANE_LOST on purpose: it is a slow-down, and a car that has lost
the lane should stop, not keep driving at half speed.
"""
import math
from dataclasses import dataclass, field, replace
from typing import List, Optional, Tuple

import config

DRIVE = "DRIVE"
CAUTION = "CAUTION"
STOP_OBSTACLE = "STOP_OBSTACLE"
STOP_RED = "STOP_RED"
STOP_SIGN = "STOP_SIGN"
STOP_LANE_LOST = "STOP_LANE_LOST"
ESTOP = "ESTOP"

STOP_STATES = (ESTOP, STOP_OBSTACLE, STOP_RED, STOP_SIGN, STOP_LANE_LOST)


@dataclass(frozen=True)
class DecisionInputs:
    t: float                                   # seconds, monotonic
    offset_px: Optional[float]                 # lane center - image center; None = no lane
    heading: Optional[float] = None            # lane heading, radians (> 0 = lane runs right ahead); None = unknown
    detections: List[dict] = field(default_factory=list)
    det_t: Optional[float] = None              # capture time of the frame YOLO ran on
    tof_mm: Optional[float] = None             # inf = no target in range; None = no valid reading
    tof_t: Optional[float] = None
    us_fl_mm: Optional[float] = None
    us_fl_t: Optional[float] = None
    us_fr_mm: Optional[float] = None
    us_fr_t: Optional[float] = None
    estop: bool = False


@dataclass(frozen=True)
class DecisionState:
    # lane / steering
    lost_frames: int = 0
    prev_offset: Optional[float] = None
    prev_t: Optional[float] = None
    last_steer: float = 0.0
    # safety net: consecutive close readings (counted per new reading, not per frame)
    last_tof_t: Optional[float] = None
    tof_close: int = 0
    last_us_key: Tuple = (None, None)
    us_close: int = 0
    # obstacle latch
    obstacle_active: bool = False
    last_stop_cond_t: Optional[float] = None
    # YOLO confirmation counters (counted per new YOLO check)
    last_det_t: Optional[float] = None
    red_count: int = 0
    green_count: int = 0
    sign_count: int = 0
    red_height: float = 0.0
    last_signal_t: Optional[float] = None
    # traffic latches
    red_stop: bool = False
    stop_sign_until: Optional[float] = None
    stop_sign_ignore_until: Optional[float] = None


@dataclass(frozen=True)
class DecisionOutput:
    state: str
    left: float
    right: float
    steer: float = 0.0
    reason: str = ""


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _fresh(value, ts, t, max_age):
    """value if it was measured within max_age, else None. ts None + value = measured now."""
    if value is None:
        return None
    if ts is not None and t - ts > max_age:
        return None
    return value


def _eff_t(value, ts, t):
    return ts if ts is not None else (t if value is not None else None)


def _cx(d, cfg):
    if "cx_ratio" in d:
        return d["cx_ratio"]
    x1, _, x2, _ = d["box"]
    return (x1 + x2) / 2.0 / cfg.CAM_WIDTH


def _bottom(d, cfg):
    if "bottom_ratio" in d:
        return d["bottom_ratio"]
    return d["box"][3] / cfg.CAM_HEIGHT


def in_corridor(d, cfg=config):
    lo, hi = cfg.CORRIDOR_X
    return lo <= _cx(d, cfg) <= hi and _bottom(d, cfg) >= cfg.CORRIDOR_BOTTOM_MIN


def _in_signal_region(d, cfg):
    lo, hi = cfg.SIGNAL_REGION_X
    return lo <= _cx(d, cfg) <= hi


def _fmt(v):
    return "inf" if v == math.inf else f"{v:.0f}"


def obstacle_eval(dets, tof, cfg=config):
    """Corridor obstacle rule. Returns (stop_reason or None, caution_reason or None)."""
    corr = [d for d in dets if d.get("cls") in cfg.OBSTACLE_CLASSES and in_corridor(d, cfg)]
    if not corr:
        return None, None
    d = max(corr, key=lambda x: x.get("box_height_ratio", 0.0))
    h = d.get("box_height_ratio", 0.0)
    if tof is not None:
        if tof < cfg.STOP_DISTANCE_MM:
            return f"{d['cls']} tof={_fmt(tof)}", None
        if tof < cfg.CAUTION_DISTANCE_MM:
            return None, f"{d['cls']} tof={_fmt(tof)}"
    elif h > cfg.STOP_BOX_HEIGHT_RATIO:
        return f"{d['cls']} h={h:.2f} (no tof)", None
    if h >= cfg.CAUTION_BOX_HEIGHT_RATIO:
        return None, f"{d['cls']} h={h:.2f}"
    return None, None


def resume_distance_ok(tof, fl, fr, cfg=config):
    """Distance half of the resume hysteresis. Returns (ok, text)."""
    need = cfg.STOP_DISTANCE_MM + cfg.RESUME_MARGIN_MM
    if tof is not None:
        return tof > need, f"tof={_fmt(tof)}>{need}"
    us = [v for v in (fl, fr) if v is not None]
    if us:
        return min(us) > need, f"us={min(us):.0f}>{need}"
    return True, "no distance sensor"


def steering(offset, prev_offset, dt, cfg=config, heading=None):
    """PD term on offset (px), plus STEER_KH x heading (rad) when known.
    Derivative in px/s; ignored if dt is invalid or too long."""
    d = 0.0
    if prev_offset is not None and dt is not None and 0 < dt <= 0.5:
        d = (offset - prev_offset) / dt
    s = cfg.STEER_KP * offset + cfg.STEER_KD * d
    kh = getattr(cfg, "STEER_KH", 0.0)
    if kh and heading is not None:
        s += kh * heading
    return _clamp(s, -cfg.STEER_MAX, cfg.STEER_MAX)


def wheel_speeds(steer, cfg=config):
    base = cfg.BASE_SPEED
    slow = getattr(cfg, "STEER_SLOWDOWN", 0.0)
    if slow and cfg.STEER_MAX > 0:
        base = base * (1.0 - slow * min(abs(steer) / cfg.STEER_MAX, 1.0))
    left = _clamp(base + steer, cfg.MIN_WHEEL_SPEED, cfg.MAX_SPEED)
    right = _clamp(base - steer, cfg.MIN_WHEEL_SPEED, cfg.MAX_SPEED)
    return left, right


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def decide(inp: DecisionInputs, st: DecisionState, cfg=config):
    """Returns (DecisionOutput, new DecisionState). Does not modify st."""
    t = inp.t
    lane_seen = inp.offset_px is not None
    lost = 0 if lane_seen else st.lost_frames + 1

    # --- freshness ---------------------------------------------------------
    det_t = _eff_t(inp.detections or None, inp.det_t, t)
    dets_fresh = det_t is not None and t - det_t <= cfg.DET_MAX_AGE_S
    dets = list(inp.detections) if dets_fresh else []
    tof = _fresh(inp.tof_mm, inp.tof_t, t, cfg.SENSOR_STALE_S)
    fl = _fresh(inp.us_fl_mm, inp.us_fl_t, t, cfg.SENSOR_STALE_S)
    fr = _fresh(inp.us_fr_mm, inp.us_fr_t, t, cfg.SENSOR_STALE_S)

    # --- safety net: consecutive close readings ------------------------------
    tof_t = _eff_t(inp.tof_mm, inp.tof_t, t)
    tof_is_close = tof is not None and tof < cfg.SAFETY_STOP_MM
    tof_close = st.tof_close
    if tof_t != st.last_tof_t:
        tof_close = tof_close + 1 if tof_is_close else 0
    if not tof_is_close:
        tof_close = 0

    us_key = (_eff_t(inp.us_fl_mm, inp.us_fl_t, t), _eff_t(inp.us_fr_mm, inp.us_fr_t, t))
    us_both_close = fl is not None and fr is not None and fl < cfg.SAFETY_STOP_MM and fr < cfg.SAFETY_STOP_MM
    us_close = st.us_close
    if us_key != st.last_us_key:
        us_close = us_close + 1 if us_both_close else 0
    if not us_both_close:
        us_close = 0

    safety_reason = None
    if tof_close >= cfg.SAFETY_CONSECUTIVE:
        safety_reason = f"safety tof={_fmt(tof)} x{tof_close}"
    elif us_close >= cfg.SAFETY_CONSECUTIVE:
        safety_reason = f"safety us={fl:.0f}/{fr:.0f} x{us_close}"

    # --- YOLO confirmation counters (once per new YOLO check) -----------------
    red_count, green_count, sign_count = st.red_count, st.green_count, st.sign_count
    red_h, last_signal_t = st.red_height, st.last_signal_t
    if not dets_fresh:
        red_count = green_count = sign_count = 0
        red_h = 0.0
    elif det_t != st.last_det_t:
        reds = [d for d in dets if d.get("cls") == cfg.RED_LIGHT_CLASS
                and d.get("conf", 0.0) >= cfg.SIGNAL_CONF and _in_signal_region(d, cfg)]
        greens = [d for d in dets if d.get("cls") == cfg.GREEN_LIGHT_CLASS
                  and d.get("conf", 0.0) >= cfg.SIGNAL_CONF and _in_signal_region(d, cfg)]
        signs = [d for d in dets if d.get("cls") == cfg.STOP_SIGN_CLASS
                 and d.get("conf", 0.0) >= cfg.STOP_SIGN_CONF
                 and d.get("box_height_ratio", 0.0) >= cfg.STOP_SIGN_HEIGHT_RATIO]
        red_seen = bool(reds)
        green_seen = bool(greens) and not red_seen          # red + green = red
        red_count = red_count + 1 if red_seen else 0
        green_count = green_count + 1 if green_seen else 0
        sign_count = sign_count + 1 if signs else 0
        red_h = max((d.get("box_height_ratio", 0.0) for d in reds), default=0.0)
        if reds or greens:
            last_signal_t = t

    # --- obstacles (with resume hysteresis) -----------------------------------
    stop_reason, obstacle_caution = obstacle_eval(dets, tof, cfg)
    cond_reason = safety_reason or stop_reason
    obstacle_active, last_cond_t = st.obstacle_active, st.last_stop_cond_t
    obstacle_reason = ""
    if cond_reason:
        obstacle_active, last_cond_t, obstacle_reason = True, t, cond_reason
    elif obstacle_active:
        clear_for = t - last_cond_t
        dist_ok, dist_txt = resume_distance_ok(tof, fl, fr, cfg)
        if clear_for >= cfg.OBSTACLE_CLEAR_S and dist_ok:
            obstacle_active = False
        elif clear_for < cfg.OBSTACLE_CLEAR_S:
            obstacle_reason = f"clearing {clear_for:.2f}s"
        else:
            obstacle_reason = f"waiting for {dist_txt}"

    # --- red light -------------------------------------------------------------
    red_stop = st.red_stop
    red_confirmed = red_count >= cfg.SIGNAL_CONFIRM_CHECKS
    red_reason = ""
    if red_confirmed and red_h >= cfg.RED_LIGHT_HEIGHT_RATIO:
        red_stop, red_reason = True, f"red light h={red_h:.2f}"
    elif red_stop:
        if green_count >= cfg.SIGNAL_CONFIRM_CHECKS:
            red_stop = False
        elif last_signal_t is not None and t - last_signal_t >= cfg.RED_TIMEOUT_S:
            red_stop = False
        else:
            since = 0.0 if last_signal_t is None else t - last_signal_t
            red_reason = f"red: waiting for green (green x{green_count}, no signal {since:.1f}s)"
    red_caution = (not red_stop) and red_confirmed and red_h < cfg.RED_LIGHT_HEIGHT_RATIO

    # --- stop sign -------------------------------------------------------------
    sign_until, sign_ignore = st.stop_sign_until, st.stop_sign_ignore_until
    if sign_until is not None and t >= sign_until:
        sign_ignore, sign_until = sign_until + cfg.STOP_SIGN_IGNORE_S, None
    sign_new = False
    if (sign_until is None and sign_count >= cfg.STOP_SIGN_CONFIRM_CHECKS
            and (sign_ignore is None or t >= sign_ignore)):
        sign_until, sign_new = t + cfg.STOP_SIGN_WAIT_S, True

    # --- new state (everything except steering) --------------------------------
    base = replace(st, lost_frames=lost,
                   last_tof_t=tof_t, tof_close=tof_close, last_us_key=us_key, us_close=us_close,
                   obstacle_active=obstacle_active, last_stop_cond_t=last_cond_t,
                   last_det_t=det_t if dets_fresh else st.last_det_t,
                   red_count=red_count, green_count=green_count, sign_count=sign_count,
                   red_height=red_h, last_signal_t=last_signal_t, red_stop=red_stop,
                   stop_sign_until=sign_until, stop_sign_ignore_until=sign_ignore)
    stopped = replace(base, prev_offset=None, prev_t=None, last_steer=0.0)

    # --- priority ----------------------------------------------------------------
    if inp.estop:
        return DecisionOutput(ESTOP, 0.0, 0.0, reason="estop"), stopped
    if obstacle_active:
        return DecisionOutput(STOP_OBSTACLE, 0.0, 0.0, reason=obstacle_reason), stopped
    if red_stop:
        return DecisionOutput(STOP_RED, 0.0, 0.0, reason=red_reason), stopped
    if sign_until is not None:
        reason = "stop sign" if sign_new else f"stop sign {sign_until - t:.1f}s left"
        return DecisionOutput(STOP_SIGN, 0.0, 0.0, reason=reason), stopped
    if lost >= cfg.LANE_LOST_FRAMES:
        return DecisionOutput(STOP_LANE_LOST, 0.0, 0.0, reason=f"lost {lost} frames"), stopped

    if lane_seen:
        dt = None if st.prev_t is None else t - st.prev_t
        steer = steering(inp.offset_px, st.prev_offset, dt, cfg, heading=inp.heading)
        reason, new_prev, new_t = "lane", inp.offset_px, t
    else:
        steer = st.last_steer   # short dropout: keep the last command
        reason, new_prev, new_t = f"dropout {lost}", None, None

    left, right = wheel_speeds(steer, cfg)
    state = DRIVE
    if obstacle_caution or red_caution:
        state = CAUTION
        left, right = left * cfg.CAUTION_FACTOR, right * cfg.CAUTION_FACTOR
        reason = obstacle_caution or f"red far h={red_h:.2f}"
    return (DecisionOutput(state, left, right, steer=steer, reason=reason),
            replace(base, prev_offset=new_prev, prev_t=new_t, last_steer=steer))
