"""
Full control loop: camera -> lane (every frame) + YOLO (every N frames) + sensors -> decision -> motors.

    python main_demo.py --dry-run                     # everything except the motors; prints commands
    python main_demo.py                               # real driving
    python main_demo.py --save-video out.mp4          # overlay video
    python main_demo.py --no-yolo                     # lane + distance safety net only
    python main_demo.py --video clip.mp4              # laptop: run on a recorded clip (implies --dry-run)
    python main_demo.py --lane ufld --dry-run         # lane backend: hough (default, config.LANE_BACKEND) or ufld

Threaded runtime (runtime.py: camera / lane / detection threads + fixed-rate control loop):
    python main_demo.py --threaded --dry-run --lane ufld          # headless; g = START, s = STOP
    python main_demo.py --threaded --source FOLDER_OR_VIDEO       # replay at REPLAY_FPS, forces dry run
    python main_demo.py --web                                     # demo page on port 8001 (starts STOPPED, DRY RUN)
Without --threaded / --web / --source everything below runs exactly as before.

Keys (in the terminal, or in the window with --show):
    q        quit (motors stop)
    space/e  E-STOP (latched)
    r        reset E-STOP
Ctrl+C always stops the motors. Any exception in the loop stops the motors, is written to the
CSV as state EXCEPTION, and the program exits cleanly (exit code 1).
"""
import argparse
import csv
import os
import sys
import time
import traceback

import cv2

import config
from camera import open_camera
from decision import DecisionInputs, DecisionState, decide
from keys import KeyReader
from lanes import BACKENDS, make_lane, steer_cfg as lane_steer_cfg
from obstacle_yolo import draw_detections, draw_regions, format_detections

STATE_COLORS = {"DRIVE": (0, 200, 0), "CAUTION": (0, 255, 255), "STOP_OBSTACLE": (0, 0, 255),
                "STOP_RED": (0, 0, 255), "STOP_SIGN": (0, 0, 200), "STOP_LANE_LOST": (0, 165, 255),
                "ESTOP": (255, 0, 255), "EXCEPTION": (255, 0, 255)}

CSV_HEADER = ["time", "frame", "offset", "detections", "det_age", "tof", "tof_age",
              "us_fl", "us_fl_age", "us_fr", "us_fr_age", "sensors_alive",
              "state", "reason", "left", "right", "fps"]


def parse_args(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--dry-run", action="store_true", help="no motors; print commands")
    p.add_argument("--no-yolo", action="store_true", help="skip obstacle detection")
    p.add_argument("--no-sensors", action="store_true", help="skip ToF/ultrasonic")
    p.add_argument("--save-video", metavar="OUT.mp4", help="save overlay video")
    p.add_argument("--show", action="store_true", help="show window (needs a desktop/VNC)")
    p.add_argument("--video", metavar="SRC", help="file/image/webcam index instead of picamera2 (forces --dry-run)")
    p.add_argument("--max-frames", type=int, default=0, help="stop after N frames (0 = run forever)")
    p.add_argument("--quiet", action="store_true", help="don't print a line per frame")
    p.add_argument("--lane", choices=BACKENDS, default=None,
                   help=f"lane backend (default config.LANE_BACKEND = {config.LANE_BACKEND!r})")
    p.add_argument("--ufld-model", choices=sorted(config.UFLD_MODELS), default=None,
                   help=f"UFLD model (default config.UFLD_MODEL = {config.UFLD_MODEL!r})")
    t = p.add_argument_group("threaded runtime")
    t.add_argument("--threaded", action="store_true", help="camera/lane/detection threads + fixed-rate loop")
    t.add_argument("--web", action="store_true", help="demo page on port 8001 (implies --threaded)")
    t.add_argument("--port", type=int, default=8001)
    t.add_argument("--source", metavar="SRC", help="replay a folder of images or a video (implies --threaded, dry run)")
    t.add_argument("--replay-fps", type=float, default=None, help="replay frame rate (default config.REPLAY_FPS)")
    t.add_argument("--hz", type=float, default=None, help="control loop rate (default config.CONTROL_HZ)")
    t.add_argument("--ufld-threads", type=int, default=None)
    t.add_argument("--yolo-threads", type=int, default=None)
    t.add_argument("--yolo-every", type=int, default=None)
    t.add_argument("--phase", default=None, help="P1..P7 (default P7 = all rules)")
    t.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = forever)")
    t.add_argument("--profile", choices=sorted(config.RUNTIME_PROFILES), default=None,
                   help="lane model + threads + YOLO on/off from config.RUNTIME_PROFILES; explicit flags win")
    return apply_profile(p.parse_args(argv))


def apply_profile(args):
    """Fill in the settings of --profile that were not given explicitly on the command line."""
    if not args.profile:
        return args
    prof = config.RUNTIME_PROFILES[args.profile]
    for key in ("lane", "ufld_model", "ufld_threads", "yolo_threads", "yolo_every"):
        if getattr(args, key) is None and key in prof:
            setattr(args, key, prof[key])
    if not prof.get("yolo", True):
        args.no_yolo = True
    return args


def fmt(v, spec=".0f"):
    return "-" if v is None else format(v, spec)


def age(now, t):
    return None if t is None else now - t


def draw_hud(img, out, offset, tof, us, fps, n_dets, yolo_on, offset_spec="+.0f"):
    col = STATE_COLORS.get(out.state, (255, 255, 255))
    cv2.rectangle(img, (0, 0), (img.shape[1], 58), (0, 0, 0), -1)
    cv2.putText(img, f"{out.state}  {out.reason}", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
    line2 = (f"L {out.left:+5.1f} R {out.right:+5.1f} | off {fmt(offset, offset_spec)} | "
             f"ToF {fmt(tof)}mm | US {fmt(us[0])}/{fmt(us[1])} | "
             f"{fps:4.1f} FPS | yolo {'on' if yolo_on else 'off'} ({n_dets})")
    cv2.putText(img, line2, (8, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return img


class _NoKeys:
    def get(self):
        return ""


def control_loop(cam, lane, detector, sensors, motors, log, *, keys=None, writer=None,
                 show=False, draw=False, verbose=False, max_frames=0, clock=time.monotonic, steer_cfg=None):
    """Runs until q, end of source, max_frames, Ctrl+C, or an exception.
    steer_cfg: None = decide() with the config module (Hough); else a config view with other steering gains.
    Returns 'quit', 'end', 'max_frames', 'interrupt' or 'exception'.
    The motors are stopped on every exit path."""
    keys = keys or _NoKeys()
    st = DecisionState()
    detections, det_t = [], None
    estop = False
    fps = 0.0
    t_start = clock()
    t_prev = None
    frame_i = 0
    warned_dead = False
    last_row = {}
    off_csv, off_hud = (".1f", "+.0f") if steer_cfg is None else (".3f", "+.2f")   # px vs normalised

    def write_row(now, state, reason, left, right):
        log.writerow([f"{now - t_start:.3f}", frame_i, fmt(last_row.get("offset"), off_csv),
                      format_detections(detections), fmt(age(now, det_t), ".3f"),
                      fmt(last_row.get("tof")), fmt(age(now, last_row.get("tof_t")), ".3f"),
                      fmt(last_row.get("fl")), fmt(age(now, last_row.get("fl_t")), ".3f"),
                      fmt(last_row.get("fr")), fmt(age(now, last_row.get("fr_t")), ".3f"),
                      last_row.get("alive", "-"), state, reason, f"{left:.1f}", f"{right:.1f}", f"{fps:.1f}"])

    try:
        while True:
            frame = cam.read()
            if frame is None:
                print("End of video source.")
                return "end"
            now = clock()

            offset, overlay = lane.process(frame, draw=draw)

            if detector is not None and frame_i % config.YOLO_EVERY_N == 0:
                detections, det_t = detector.detect(frame), now   # reused until the next YOLO frame

            if sensors is not None:
                tof, tof_t = sensors.tof_reading()
                (fl, fl_t), (fr, fr_t) = sensors.ultrasonic_readings()
                alive = sensors.alive()
                if not alive and not warned_dead:
                    print("[main] WARNING: sensor thread not running; distance readings will expire")
                    warned_dead = True
            else:
                tof = tof_t = fl = fl_t = fr = fr_t = None
                alive = "-"
            last_row = dict(offset=offset, tof=tof, tof_t=tof_t, fl=fl, fl_t=fl_t, fr=fr, fr_t=fr_t, alive=alive)

            inp = DecisionInputs(t=now, offset_px=offset, detections=detections, det_t=det_t,
                                 tof_mm=tof, tof_t=tof_t, us_fl_mm=fl, us_fl_t=fl_t,
                                 us_fr_mm=fr, us_fr_t=fr_t, estop=estop)
            out, st = decide(inp, st) if steer_cfg is None else decide(inp, st, steer_cfg)
            motors.drive(out.left, out.right)     # (0, 0) = active brake, then coast

            if t_prev is not None and now > t_prev:
                inst = 1.0 / (now - t_prev)
                fps = inst if fps == 0 else 0.9 * fps + 0.1 * inst
            t_prev = now

            write_row(now, out.state, out.reason, out.left, out.right)

            if verbose:
                print(f"[{frame_i:5d}] {out.state:14s} L={out.left:+5.1f} R={out.right:+5.1f} "
                      f"off={fmt(offset, '+.0f'):>5} tof={fmt(tof):>5} dets={len(detections)} "
                      f"{fps:4.1f}fps  {out.reason}")

            if draw:
                vis = draw_regions(draw_detections(overlay, detections))
                us_now = (fl, fr)
                draw_hud(vis, out, offset, tof, us_now, fps, len(detections), detector is not None, off_hud)
                if writer is not None:
                    writer.write(vis)
                if show:
                    cv2.imshow("car", vis)

            key = keys.get()
            if show:
                k = cv2.waitKey(1) & 0xFF
                if k != 255:
                    key = chr(k)
            if key == "q":
                print("q pressed: stopping.")
                return "quit"
            if key in (" ", "e") and not estop:
                estop = True
                print("E-STOP latched (r to reset)")
            elif key == "r" and estop:
                estop = False
                print("E-STOP reset")

            frame_i += 1
            if max_frames and frame_i >= max_frames:
                return "max_frames"

    except KeyboardInterrupt:
        print("\nCtrl+C: stopping.")
        motors.stop()
        return "interrupt"
    except Exception as e:
        # Stop first, then record what happened
        try:
            motors.stop()
        finally:
            now = clock()
            print(f"\n[main] EXCEPTION in control loop: {e!r} -> motors stopped")
            traceback.print_exc()
            try:
                write_row(now, "EXCEPTION", f"{type(e).__name__}: {e}"[:200], 0.0, 0.0)
            except Exception:
                pass
        return "exception"
    finally:
        motors.drive(0, 0)


def main(argv=None):
    args = parse_args(argv)
    if args.threaded or args.web or args.source:
        import run_threaded
        return run_threaded.main(args)
    if args.video:
        args.dry_run = True
    draw = bool(args.save_video or args.show)

    # Initialize motors first so the SIGINT handler is in place as early as possible
    if args.dry_run:
        from motors import DryRunMotors
        motors = DryRunMotors(verbose=False)
    else:
        from motors import Motors
        motors = Motors()

    from fans import Fans
    fans = Fans()      # no-op if the GPIO is unavailable (laptop)
    fans.on()

    cam = sensors = writer = log_f = None
    result = "exception"
    try:
        cam = open_camera(args.video)
        backend = args.lane or config.LANE_BACKEND
        lane = make_lane(backend, model=args.ufld_model)
        print(f"Lane backend: {lane.source}")

        detector = None
        if not args.no_yolo:
            from obstacle_yolo import ObstacleDetector
            print("Loading YOLO...")
            detector = ObstacleDetector()

        if not args.no_sensors:
            from sensors import Sensors
            sensors = Sensors(use_gyro=False).start()

        if args.save_video:
            writer = cv2.VideoWriter(args.save_video, cv2.VideoWriter_fourcc(*"mp4v"),
                                     config.VIDEO_FPS, (config.CAM_WIDTH, config.CAM_HEIGHT))

        os.makedirs(config.LOG_DIR, exist_ok=True)
        log_path = os.path.join(config.LOG_DIR, time.strftime("run_%Y%m%d_%H%M%S.csv"))
        log_f = open(log_path, "w", newline="")
        log = csv.writer(log_f)
        log.writerow(CSV_HEADER)
        print(f"Logging to {log_path}")
        print("Running. q = quit, space/e = E-STOP, r = reset, Ctrl+C = stop.")

        with KeyReader() as keys:
            result = control_loop(cam, lane, detector, sensors, motors, log, keys=keys, writer=writer,
                                  show=args.show, draw=draw, verbose=args.dry_run and not args.quiet,
                                  max_frames=args.max_frames, steer_cfg=lane_steer_cfg(backend))
    except KeyboardInterrupt:
        print("\nCtrl+C: stopping.")
        result = "interrupt"
    except Exception as e:
        print(f"[main] startup error: {e!r}")
        traceback.print_exc()
        result = "exception"
    finally:
        try:
            motors.stop()
        finally:
            motors.cleanup()
            for closer in (fans.cleanup,
                           lambda: cam and cam.close(),
                           lambda: sensors and sensors.close(),
                           lambda: writer and writer.release(),
                           lambda: log_f and log_f.close(),
                           lambda: args.show and cv2.destroyAllWindows()):
                try:
                    closer()
                except Exception as e:
                    print(f"cleanup: {e}")
            if args.save_video:
                print(f"Video saved to {args.save_video}")
    return 1 if result == "exception" else 0


if __name__ == "__main__":
    sys.exit(main())
