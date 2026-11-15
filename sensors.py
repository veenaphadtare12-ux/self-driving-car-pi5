"""
Front distance sensors. Every reader returns None on failure and never raises.

  * VL53L5CX ToF (vl53l5cx_ctypes, same init as ~/robotex_ai/robot_hardware.py, plus the
    ranging frequency): minimum distance in mm over the center zones.
    math.inf = fresh frame with no target in the center zones (clear);
    None = no valid reading (unknown).
  * HC-SR04 ultrasonics front-left / front-right via lgpio, triggered alternately
    (never together, at least US_GAP_S apart): distance in mm, None on timeout or error.
  * MPU6050 (0x68): gyro z in rad/s (optional, informational).

    s = Sensors().start()      # background thread; the main loop never blocks on sensors
    val, t = s.tof_reading()   # raw value + monotonic time it was measured (decision.py applies age)
    s.tof_min_mm()             # value, or None if older than SENSOR_STALE_S (for test scripts)
    s.alive()                  # False if the thread died or has not completed a cycle recently
    s.close()
"""
import math
import threading
import time

import config


def _log(msg):
    print(f"[sensors] {msg}")


# ---------------------------------------------------------------------------
# VL53L5CX
# ---------------------------------------------------------------------------
class FrontToF:
    def __init__(self):
        self.ok = False
        self.vl53 = None
        self._center_idx = [r * 8 + c for r in config.TOF_CENTER_ROWS for c in config.TOF_CENTER_COLS]
        try:
            import vl53l5cx_ctypes as vl53l5cx
            _log("VL53L5CX init (firmware upload takes ~10 s)...")
            self.vl53 = vl53l5cx.VL53L5CX()
            self.vl53.init()
            self.vl53.set_resolution(config.TOF_RESOLUTION)
            try:
                # Default is 1 Hz: far too slow for stopping
                self.vl53.set_ranging_frequency_hz(config.TOF_RANGING_HZ)
                _log(f"VL53L5CX ranging at {config.TOF_RANGING_HZ} Hz")
            except Exception as e:
                _log(f"WARNING: could not set ranging frequency ({e}); ToF may run at 1 Hz")
            self.vl53.start_ranging()
            self.ok = True
            _log("VL53L5CX OK")
        except Exception as e:
            _log(f"VL53L5CX unavailable: {e}")

    def read_zones(self):
        """Returns (distances[64], statuses[64] or None, n_targets[64] or None), or None
        if there is no new frame or on error."""
        if not self.ok:
            return None
        try:
            if not self.vl53.data_ready():
                return None
            data = self.vl53.get_data()
            # 2D ctypes array (1 x 64): take [0] first (as in robot_hardware.py)
            dist = list(data.distance_mm[0])
            try:
                status = list(data.target_status[0])
            except Exception:
                status = None
            try:
                nt = data.nb_target_detected
                try:
                    nt[0][0]        # 2-D like distance_mm -> take the inner array
                    nt = nt[0]
                except TypeError:
                    pass
                n_targets = list(nt)
                if len(n_targets) < 64:
                    n_targets = None
            except Exception:
                n_targets = None
            return dist, status, n_targets
        except Exception as e:
            _log(f"ToF read error: {e}")
            return None

    def center_min_mm(self, zones):
        """min valid distance in the center zones; inf if they all report no target; else None."""
        if zones is None:
            return None
        dist, status, n_targets = zones
        vals = []
        for i in self._center_idx:
            d = dist[i]
            if status is not None and status[i] not in config.TOF_VALID_STATUSES:
                continue
            if config.TOF_MIN_VALID_MM <= d <= config.TOF_MAX_VALID_MM:
                vals.append(d)
        if vals:
            return float(min(vals))
        if n_targets is not None and all(n_targets[i] == 0 for i in self._center_idx):
            return math.inf     # sensor working, nothing in range
        return None             # targets present but none valid: unknown

    def close(self):
        if self.ok:
            try:
                self.vl53.stop_ranging()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# HC-SR04 via lgpio
# ---------------------------------------------------------------------------
class Ultrasonic:
    """Uses lgpio edge alerts (kernel timestamps). Falls back to polling with gpio_read.
    The trig pin must already be claimed as an output (Sensors does it via open_and_claim)."""

    def __init__(self, handle, trig, echo):
        import lgpio
        self.lg, self.h, self.trig, self.echo = lgpio, handle, trig, echo
        self.ok = False
        self._cb = None
        self._armed = False
        self._rise = None
        self._dur_ns = None
        self._evt = threading.Event()
        try:
            try:
                lgpio.gpio_claim_alert(handle, echo, lgpio.BOTH_EDGES)
                self._cb = lgpio.callback(handle, echo, lgpio.BOTH_EDGES, self._edge)
                self.mode = "alert"
            except Exception:
                lgpio.gpio_claim_input(handle, echo)
                self.mode = "poll"
            self.ok = True
        except Exception as e:
            _log(f"ultrasonic {trig}/{echo} unavailable: {e}")

    def _edge(self, chip, gpio, level, tick_ns):
        if not self._armed:
            return              # late echo from an earlier ping: ignore
        if level == 1:
            self._rise = tick_ns
        elif level == 0 and self._rise is not None:
            self._dur_ns = tick_ns - self._rise
            self._armed = False
            self._evt.set()

    def read_mm(self):
        if not self.ok:
            return None
        try:
            dur_s = self._measure_alert() if self.mode == "alert" else self._measure_poll()
            if dur_s is None:
                return None
            mm = dur_s * 343_000.0 / 2.0
            return mm if config.US_MIN_MM <= mm <= config.US_MAX_MM else None
        except Exception as e:
            _log(f"ultrasonic {self.trig}/{self.echo} read error: {e}")
            return None

    def _pulse(self):
        self.lg.gpio_write(self.h, self.trig, 1)
        time.sleep(0.00001)
        self.lg.gpio_write(self.h, self.trig, 0)

    def _measure_alert(self):
        self._rise, self._dur_ns = None, None
        self._evt.clear()
        self._armed = True
        self._pulse()
        got = self._evt.wait(config.US_TIMEOUT_S + 0.005)
        self._armed = False
        return self._dur_ns / 1e9 if got and self._dur_ns is not None else None

    def _measure_poll(self):
        lg, h, echo = self.lg, self.h, self.echo
        self._pulse()
        deadline = time.perf_counter() + config.US_TIMEOUT_S
        while lg.gpio_read(h, echo) == 0:
            if time.perf_counter() > deadline:
                return None
        start = time.perf_counter()
        while lg.gpio_read(h, echo) == 1:
            if time.perf_counter() > deadline:
                return None
        return time.perf_counter() - start

    def close(self):
        try:
            if self._cb is not None:
                self._cb.cancel()
            self.lg.gpio_free(self.h, self.trig)
            self.lg.gpio_free(self.h, self.echo)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# MPU6050
# ---------------------------------------------------------------------------
class Gyro:
    def __init__(self):
        self.ok = False
        try:
            import mpu6050
            self.dev = mpu6050.mpu6050(config.MPU6050_ADDR)
            self.ok = True
        except Exception as e:
            _log(f"MPU6050 unavailable: {e}")

    def read_z(self):
        if not self.ok:
            return None
        try:
            return self.dev.get_gyro_data()["z"] - config.GYRO_Z_OFFSET
        except Exception:
            return None


# ---------------------------------------------------------------------------
# Aggregate with a background thread
# ---------------------------------------------------------------------------
class Sensors:
    def __init__(self, use_tof=True, use_ultrasonic=True, use_gyro=True):
        self.tof = FrontToF() if use_tof else None
        self.gyro = Gyro() if use_gyro else None
        self._h = None
        self.us_fl = self.us_fr = None
        if use_ultrasonic:
            try:
                from gpio_chip import open_and_claim
                # Claiming both trig pins picks the chip (gpiochip0, else gpiochip4)
                _, self._h, _ = open_and_claim({config.US_FRONT_LEFT[0]: 0, config.US_FRONT_RIGHT[0]: 0},
                                               "ultrasonic")
                self.us_fl = Ultrasonic(self._h, *config.US_FRONT_LEFT)
                self.us_fr = Ultrasonic(self._h, *config.US_FRONT_RIGHT)
            except Exception as e:
                _log(f"ultrasonics unavailable: {e}")

        self._lock = threading.Lock()
        # key -> (value, monotonic time measured). Time None = never measured.
        self._vals = {"tof": (None, None), "fl": (None, None), "fr": (None, None), "gyro": (None, None)}
        self._stop = threading.Event()
        self._thread = None
        self._last_cycle = None
        self._last_ping = 0.0
        self._next_us = "fl"

    # -- synchronous reads (used by the test/calibration scripts) --------------
    def read_tof_now(self, wait_s=0.2):
        """Blocks up to wait_s for a fresh frame. Returns mm, inf (no target) or None."""
        if self.tof is None:
            return None
        end = time.monotonic() + wait_s
        while time.monotonic() < end:
            z = self.tof.read_zones()
            if z is not None:
                return self.tof.center_min_mm(z)
            time.sleep(0.01)
        return None

    def _ping(self, sensor):
        """One ultrasonic reading, at least US_GAP_S after the previous ping of either sensor."""
        if sensor is None:
            return None
        wait = config.US_GAP_S - (time.monotonic() - self._last_ping)
        if wait > 0:
            time.sleep(wait)
        try:
            return sensor.read_mm()
        finally:
            self._last_ping = time.monotonic()

    def read_ultrasonic_now(self):
        """Both sensors, one after the other (never together)."""
        return self._ping(self.us_fl), self._ping(self.us_fr)

    def read_gyro_now(self):
        return self.gyro.read_z() if self.gyro else None

    # -- background -------------------------------------------------------
    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
        return self

    def _store(self, key, val):
        with self._lock:
            self._vals[key] = (val, time.monotonic())

    def _loop(self):
        period = 1.0 / config.SENSOR_POLL_HZ
        while not self._stop.is_set():
            t0 = time.monotonic()
            try:
                if self.tof is not None:
                    z = self.tof.read_zones()
                    if z is not None:
                        self._store("tof", self.tof.center_min_mm(z))
                # Alternate: one ultrasonic per cycle, so they are never triggered together
                key, sensor = ("fl", self.us_fl) if self._next_us == "fl" else ("fr", self.us_fr)
                self._next_us = "fr" if key == "fl" else "fl"
                if sensor is not None and time.monotonic() - self._last_ping >= config.US_GAP_S:
                    self._store(key, self._ping(sensor))
                elif sensor is not None:
                    self._next_us = key   # gap not over yet: try the same sensor next cycle
                if self.gyro is not None:
                    self._store("gyro", self.read_gyro_now())
                self._last_cycle = time.monotonic()
            except Exception as e:
                _log(f"sensor loop error: {e}")
            self._stop.wait(max(0.0, period - (time.monotonic() - t0)))

    def _raw(self, key):
        with self._lock:
            return self._vals[key]

    def _get(self, key):
        val, t = self._raw(key)
        if t is None or time.monotonic() - t > config.SENSOR_STALE_S:
            return None
        return val

    # raw (value, time) for decision.py, which applies the age limit itself
    def tof_reading(self):
        return self._raw("tof")

    def ultrasonic_readings(self):
        return self._raw("fl"), self._raw("fr")

    # stale-checked values for scripts
    def tof_min_mm(self):
        return self._get("tof")

    def ultrasonic_mm(self):
        return self._get("fl"), self._get("fr")

    def gyro_z(self):
        return self._get("gyro")

    def alive(self, max_cycle_s=0.5):
        """False if the thread died or hung (no completed cycle within max_cycle_s)."""
        if self._thread is None or not self._thread.is_alive():
            return False
        return self._last_cycle is not None and time.monotonic() - self._last_cycle <= max_cycle_s

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        for s in (self.us_fl, self.us_fr):
            if s:
                s.close()
        if self.tof:
            self.tof.close()
        if self._h is not None:
            try:
                import lgpio
                lgpio.gpiochip_close(self._h)
            except Exception:
                pass
