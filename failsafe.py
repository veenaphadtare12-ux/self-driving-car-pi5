"""
Fail-safes for the threaded runtime (UFLD Part 4). Evaluated every control tick by CarRuntime.tick();
they only ever STOP the car or slow it down: decide() and its rules are not touched.

  * lane / detection / camera thread crashed (runtime.py), or published nothing for FAILSAFE_STALE_S (1 s) -> STOP
  * UFLD unhealthy: lane FPS below FAILSAFE_UFLD_MIN_FPS (2.5) over the last FAILSAFE_UFLD_WINDOW_S (3 s)
    while driving -> STOP; the page offers a one-tap switch to Hough (allowed only while stopped)
  * CPU at/above FAILSAFE_TEMP_SLOW_C (80 C) -> wheel speeds x FAILSAFE_HOT_FACTOR; at/above FAILSAFE_TEMP_STOP_C (85 C) -> STOP
  * under-voltage (vcgencmd get_throttled bit 0 = now, bit 16 = since boot) appearing during the run -> logged + warning
  * demo page heartbeat missing for FAILSAFE_HEARTBEAT_S (1.5 s) while driving -> STOP (only while a page controls)

A STOP latches: the car stays stopped (running=False) until START is pressed again.
check() returns (action, text) with action None, "warn", "slow" or "stop".
"""
import time

import config

UNDERVOLT_NOW, UNDERVOLT_SINCE_BOOT = 0x1, 0x10000


class FailSafe:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.stale_s = config.FAILSAFE_STALE_S
        self.ufld_min_fps = config.FAILSAFE_UFLD_MIN_FPS
        self.ufld_window = config.FAILSAFE_UFLD_WINDOW_S
        self.temp_slow = config.FAILSAFE_TEMP_SLOW_C
        self.temp_stop = config.FAILSAFE_TEMP_STOP_C
        self.heartbeat_s = config.FAILSAFE_HEARTBEAT_S
        self.heartbeat_required = False      # the web page sets True while a controller is attached
        self.last_heartbeat = None
        self.undervolt_seen = False
        self.undervolt_at_start = None
        self.offer_hough = False
        self.events = []                     # (time, text), last 20, for the page

    # -- hooks from the runtime / web ---------------------------------------
    def heartbeat(self, now=None):
        self.last_heartbeat = self.clock() if now is None else now

    def on_start(self, rt):
        self.last_heartbeat = self.clock()
        self.offer_hough = False

    def on_lane_switch(self, rt):
        self.offer_hough = False

    def _event(self, text):
        if not self.events or self.events[-1][1] != text:
            self.events.append((time.strftime("%H:%M:%S"), text))
            self.events = self.events[-20:]

    # -- per tick --------------------------------------------------------------
    def check(self, rt, now):
        stops, slow, warn = [], None, None

        def stale(name, last, started):
            ref = last if last is not None else started
            if ref is not None and now - ref > self.stale_s:
                stops.append(f"{name} thread stale {now - ref:.1f}s")

        lw = rt.lane_worker
        stale("lane", lw.last_publish, lw.started_t)
        if rt.det_worker is not None:
            stale("detection", rt.det_worker.last_publish, rt.det_worker.started_t)
        stale("camera", rt.cam_thread.last_t, lw.started_t)

        if rt.lane_backend == "ufld" and rt.running and lw.started_t is not None \
                and now - lw.started_t >= self.ufld_window:
            fps = lw.meter.rate(now)
            if fps < self.ufld_min_fps:
                stops.append(f"UFLD unhealthy: {fps:.1f} lane fps < {self.ufld_min_fps:g} over "
                             f"{self.ufld_window:g}s - switch to Hough?")
                self.offer_hough = True

        mon = rt.monitor
        temp = None if mon is None else mon.temp_c
        if temp is not None:
            if temp >= self.temp_stop:
                stops.append(f"CPU {temp:.1f}C >= {self.temp_stop:g}C")
            elif temp >= self.temp_slow:
                slow = f"CPU {temp:.1f}C >= {self.temp_slow:g}C: half speed"

        thr = None if mon is None else mon.throttled
        if thr is not None:
            if self.undervolt_at_start is None:
                self.undervolt_at_start = bool(thr & UNDERVOLT_SINCE_BOOT)
            if thr & UNDERVOLT_NOW or (thr & UNDERVOLT_SINCE_BOOT and not self.undervolt_at_start):
                warn = f"UNDER-VOLTAGE (throttled={hex(thr)})"
                if not self.undervolt_seen:
                    self.undervolt_seen = True
                    print(f"[failsafe] WARNING {warn}")

        if self.heartbeat_required and rt.running:
            hb = self.last_heartbeat
            if hb is None or now - hb > self.heartbeat_s:
                stops.append(f"page heartbeat lost {0.0 if hb is None else now - hb:.1f}s")

        if stops:
            txt = "; ".join(stops)
            if rt.running:
                self._event(txt)
            return "stop", txt + (f"; {warn}" if warn else "")
        if slow:
            self._event(slow)
            return "slow", slow + (f"; {warn}" if warn else "")
        if warn:
            self._event(warn)
            return "warn", warn
        return None, ""
