"""
Builds and runs the threaded runtime (runtime.py) for main_demo.py --threaded / --source / --web.

Motors: always DryRunMotors at start. Real motors are only created when DRY RUN is switched off on the
web page (not possible with --source or --dry-run). Headless --threaded with real motors needs neither
--dry-run nor --source and starts driving immediately, like the old main_demo.
"""
import os
import socket
import time
import traceback

import config


def local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:                                            # noqa: BLE001
        return "127.0.0.1"


def make_detector(threads=None):
    from obstacle_yolo import ObstacleDetector
    return ObstacleDetector(threads=threads if threads is not None else config.YOLO_THREADS)


def lane_factory_for(ufld_threads=None):
    from lanes import make_lane

    def f(backend, model=None):
        return make_lane(backend, model=model, threads=ufld_threads or config.UFLD_THREADS)
    return f


class NullCamera:
    """No camera: read() never returns a frame (the camera fail-safe keeps the car stopped)."""
    connected = False

    def __init__(self):
        import threading
        self._ev = threading.Event()

    def read(self):
        while not self._ev.wait(0.5):
            pass
        return None

    def close(self):
        self._ev.set()


def open_source(source=None, replay_fps=None, allow_missing=False):
    if source:
        from camera import ReplayCamera
        return ReplayCamera(source, fps=replay_fps or config.REPLAY_FPS, loop=True)
    from camera import open_camera
    try:
        return open_camera()
    except Exception as e:                                       # noqa: BLE001
        if not allow_missing:
            raise
        print(f"[threaded] CAMERA NOT CONNECTED ({e}); page runs, car cannot drive")
        return NullCamera()


def build(args, running, log=True):
    """Returns (runtime, cleanup list). Raises on startup errors (caller stops everything)."""
    from runtime import CarRuntime, SysMonitor
    backend = args.lane or config.LANE_BACKEND
    lf = lane_factory_for(args.ufld_threads)
    lane = lf(backend, args.ufld_model)
    print(f"Lane backend: {lane.source}")
    det = None
    if not args.no_yolo:
        print("Loading YOLO...")
        det = make_detector(args.yolo_threads)
    sensors = None
    if not args.no_sensors:
        from sensors import Sensors
        sensors = Sensors(use_gyro=False).start()
    cam = open_source(args.source, args.replay_fps, allow_missing=bool(getattr(args, "web", False)))
    log_path = None
    if log:
        os.makedirs(config.LOG_DIR, exist_ok=True)
        log_path = os.path.join(config.LOG_DIR, time.strftime("run_%Y%m%d_%H%M%S.csv"))
    rt = CarRuntime(cam, lane, det, sensors, lane_backend=backend, lane_factory=lf, hz=args.hz,
                    yolo_every_n=args.yolo_every, phase=args.phase or "P7", running=running,
                    log_path=log_path, monitor=SysMonitor(),
                    verbose=bool(args.dry_run and not args.quiet and not getattr(args, "web", False)))
    try:
        from failsafe import FailSafe
        rt.failsafe = FailSafe()
    except ImportError:
        pass
    rt.allow_real_motors = not (args.dry_run or args.source)
    if log_path:
        print(f"Logging to {log_path}")
    return rt, [lambda: cam.close(), lambda: sensors and sensors.close()]


def main(args):
    if args.source:
        args.dry_run = True
    from fans import Fans
    fans = Fans()
    fans.on()
    rt, closers, srv, result = None, [], None, "exception"
    try:
        web = bool(args.web)
        rt, closers = build(args, running=False)
        if not web:
            if not args.dry_run:
                # headless real driving, like the old main_demo: real motors from the start
                from motors import Motors
                rt.set_dry_run(False, Motors)
            rt.running = True
        rt.start_threads()
        if web:
            import web as webmod
            srv = webmod.start(rt, args.port, real_motors_factory=None if not rt.allow_real_motors else _motors)
            print(f"Demo page: http://{local_ip()}:{args.port}/   (STOPPED, DRY RUN)")
        print("Running. q = quit, space/e = E-STOP, r = reset, g = START, s = STOP, Ctrl+C = stop.")
        from keys import KeyReader
        with KeyReader() as keys:
            result = rt.run(duration=args.duration, keys=keys)
        print(f"[threaded] exit: {result} | {rt.bench_summary()}")
    except KeyboardInterrupt:
        print("\nCtrl+C: stopping.")
        result = "interrupt"
    except Exception as e:                                       # noqa: BLE001
        print(f"[threaded] startup error: {e!r}")
        traceback.print_exc()
        result = "exception"
    finally:
        if rt is not None:
            try:
                rt.motors.stop()
            except Exception:                                    # noqa: BLE001
                pass
            if rt.motors is not rt.dry_motors:
                try:
                    rt.motors.cleanup()
                except Exception as e:                           # noqa: BLE001
                    print(f"cleanup: {e}")
        for c in [lambda: srv and srv.shutdown(), lambda: rt and rt.close(), fans.cleanup] + closers:
            try:
                c()
            except Exception as e:                               # noqa: BLE001
                print(f"cleanup: {e}")
    return 1 if result == "exception" else 0


def _motors():
    from motors import Motors
    return Motors()
