"""
Cooling fans on a relay (config.FAN_PIN, active-low by default). Never raises:
if the GPIO is unavailable (for example on the laptop), on()/off() do nothing.

    fans = Fans()
    fans.on()
    try:
        ...
    finally:
        fans.cleanup()     # OFF and release the pin
"""
import atexit

import config


class Fans:
    def __init__(self):
        self.ok = False
        self._on_level = 0 if config.FAN_ACTIVE_LOW else 1
        self._off_level = 1 - self._on_level
        try:
            from gpio_chip import open_and_claim
            self._lg, self._h, _ = open_and_claim({config.FAN_PIN: self._off_level}, "fans")
            self.ok = True
            atexit.register(self.cleanup)
        except Exception as e:
            print(f"[fans] unavailable, continuing without fans: {e}")

    def _write(self, level):
        if self.ok:
            try:
                self._lg.gpio_write(self._h, config.FAN_PIN, level)
            except Exception as e:
                print(f"[fans] write failed: {e}")

    def on(self):
        self._write(self._on_level)
        if self.ok:
            print("[fans] ON")

    def off(self):
        self._write(self._off_level)

    def cleanup(self):
        if not self.ok:
            return
        self.off()
        self.ok = False
        try:
            self._lg.gpio_free(self._h, config.FAN_PIN)
            self._lg.gpiochip_close(self._h)
        except Exception:
            pass
        print("[fans] OFF")
