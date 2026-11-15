"""
TB6612FNG skid-steer motor control with lgpio (Pi 5).

    m = Motors()
    try:
        m.drive(30, 30)     # -100..100 per side, clamped to config.MAX_SPEED
    finally:
        m.cleanup()         # stops motors and frees the pins

Stopping:
  * stop() = active brake (IN1=IN2=1, PWM 100 = TB6612 short brake) for config.BRAKE_S, then
    coast (IN1=IN2=0, PWM 0). Non-blocking: the watchdog thread switches to coast.
    stop(block=True) waits for the brake to finish (used by cleanup()).
  * drive(0, 0) while moving calls stop(), so every STOP state brakes.

Safety:
  * SIGINT / SIGTERM stop the motors immediately and then raise
    KeyboardInterrupt, so the caller's finally block still runs.
  * A watchdog thread stops the motors if drive() is not called for
    config.MOTOR_WATCHDOG_S (for example, if the main loop hangs). Call drive()
    every loop iteration.
  * cleanup() is also registered with atexit.
"""
import atexit
import signal
import threading
import time

import config


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Motors:
    def __init__(self):
        from gpio_chip import open_and_claim  # lgpio is imported there, so this module loads on the laptop
        # RLock: the SIGINT handler runs on the main thread and may interrupt drive()
        # while it holds the lock. A plain Lock would deadlock there.
        self._lock = threading.RLock()
        self._closed = False

        self._pins = [config.LEFT_IN1, config.LEFT_IN2, config.LEFT_PWM,
                      config.RIGHT_IN1, config.RIGHT_IN2, config.RIGHT_PWM]
        # Tries gpiochip0, then gpiochip4; prints which one is used
        self._lg, self._h, self.chip = open_and_claim({p: 0 for p in self._pins}, "motors")

        self._last_duty = {config.LEFT_PWM: None, config.RIGHT_PWM: None}
        self._last_cmd_time = time.monotonic()
        self._moving = False
        self._brake_until = None

        self._coast_locked()
        self._install_signal_handlers()
        atexit.register(self.cleanup)

        self._wd_stop = threading.Event()
        self._wd = threading.Thread(target=self._watchdog, daemon=True)
        self._wd.start()

    # -- public API ---------------------------------------------------------
    def drive(self, left, right):
        """left/right in -100..100 (percent). Clamped to +-MAX_SPEED. (0, 0) brakes, then coasts."""
        left = _clamp(float(left), -config.MAX_SPEED, config.MAX_SPEED)
        right = _clamp(float(right), -config.MAX_SPEED, config.MAX_SPEED)
        with self._lock:
            if self._closed:
                return
            self._last_cmd_time = time.monotonic()
            if left == 0 and right == 0:
                if self._moving:
                    self._brake_locked()
                return              # already braking or coasting
            self._brake_until = None
            self._set_side(left, config.LEFT_INVERT, config.LEFT_TRIM,
                           config.LEFT_IN1, config.LEFT_IN2, config.LEFT_PWM)
            self._set_side(right, config.RIGHT_INVERT, config.RIGHT_TRIM,
                           config.RIGHT_IN1, config.RIGHT_IN2, config.RIGHT_PWM)
            self._moving = (left != 0 or right != 0)

    def stop(self, block=False):
        """Active brake for BRAKE_S, then coast. block=True waits for the coast."""
        with self._lock:
            if self._closed:
                return
            self._brake_locked()
        if block:
            time.sleep(config.BRAKE_S)
            with self._lock:
                if not self._closed:
                    self._coast_locked()

    def cleanup(self):
        with self._lock:
            if self._closed:
                return
            try:
                self._brake_locked()
            except Exception as e:
                print(f"[motors] brake during cleanup failed: {e}")
        time.sleep(config.BRAKE_S)
        with self._lock:
            if self._closed:
                return
            try:
                self._coast_locked()
            except Exception as e:
                print(f"[motors] coast during cleanup failed: {e}")
            self._closed = True
        if hasattr(self, "_wd_stop"):
            self._wd_stop.set()
        try:
            for p in self._pins:
                try:
                    self._lg.gpio_free(self._h, p)
                except Exception:
                    pass
            self._lg.gpiochip_close(self._h)
        except Exception as e:
            print(f"[motors] cleanup error: {e}")
        print("[motors] stopped and released")

    # -- internals ----------------------------------------------------------
    def _brake_locked(self):
        """TB6612 short brake: IN1=IN2=1 with PWM high."""
        lg, h = self._lg, self._h
        for p in (config.LEFT_IN1, config.LEFT_IN2, config.RIGHT_IN1, config.RIGHT_IN2):
            lg.gpio_write(h, p, 1)
        for pwm in (config.LEFT_PWM, config.RIGHT_PWM):
            lg.tx_pwm(h, pwm, config.PWM_FREQ_HZ, 100)
            self._last_duty[pwm] = 100
        self._moving = False
        self._brake_until = time.monotonic() + config.BRAKE_S

    def _coast_locked(self):
        """Outputs off: IN1=IN2=0, PWM 0."""
        lg, h = self._lg, self._h
        for pwm in (config.LEFT_PWM, config.RIGHT_PWM):
            lg.tx_pwm(h, pwm, config.PWM_FREQ_HZ, 0)
            self._last_duty[pwm] = 0
        for p in (config.LEFT_IN1, config.LEFT_IN2, config.RIGHT_IN1, config.RIGHT_IN2):
            lg.gpio_write(h, p, 0)
        self._moving = False
        self._brake_until = None

    def _set_side(self, speed, invert, trim, in1, in2, pwm):
        if invert:
            speed = -speed
        speed *= trim
        lg, h = self._lg, self._h
        if speed > 0:
            lg.gpio_write(h, in1, 1)
            lg.gpio_write(h, in2, 0)
        elif speed < 0:
            lg.gpio_write(h, in1, 0)
            lg.gpio_write(h, in2, 1)
        else:
            lg.gpio_write(h, in1, 0)
            lg.gpio_write(h, in2, 0)
        duty = round(_clamp(abs(speed), 0, 100), 1)
        # Only call tx_pwm when the duty changes, so the PWM is not restarted every frame
        if duty != self._last_duty[pwm]:
            lg.tx_pwm(h, pwm, config.PWM_FREQ_HZ, duty)
            self._last_duty[pwm] = duty

    def _watchdog(self):
        while not self._wd_stop.wait(0.02):
            try:
                with self._lock:
                    if self._closed:
                        return
                    now = time.monotonic()
                    if self._brake_until is not None and now >= self._brake_until:
                        self._coast_locked()          # brake finished -> coast
                    elif self._moving and now - self._last_cmd_time > config.MOTOR_WATCHDOG_S:
                        print("[motors] WATCHDOG: no command received, stopping")
                        self._brake_locked()
            except Exception:
                pass

    def _install_signal_handlers(self):
        def handler(signum, frame):
            try:
                self.stop()
            finally:
                raise KeyboardInterrupt
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                signal.signal(sig, handler)
            except (ValueError, OSError):
                pass  # not the main thread


class DryRunMotors:
    """Same interface as Motors, but only prints. Used with --dry-run and on the laptop."""

    def __init__(self, verbose=True):
        self.verbose = verbose
        self.last = (0.0, 0.0)

    def drive(self, left, right):
        left = _clamp(float(left), -config.MAX_SPEED, config.MAX_SPEED)
        right = _clamp(float(right), -config.MAX_SPEED, config.MAX_SPEED)
        self.last = (left, right)
        if self.verbose:
            print(f"[dry-run] drive L={left:+6.1f} R={right:+6.1f}")

    def stop(self, block=False):
        self.last = (0.0, 0.0)
        if self.verbose:
            print("[dry-run] stop (brake, then coast)")

    def cleanup(self):
        self.stop()
