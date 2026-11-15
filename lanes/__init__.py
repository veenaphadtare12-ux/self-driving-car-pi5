"""Lane backends with one common result type.

    from lanes import make_lane
    lane = make_lane("ufld")            # or "hough"; default config.LANE_BACKEND
    offset, overlay = lane.process(frame_bgr, draw=True)   # offset in the backend's steering units
    r = lane.result                     # LaneResult (normalised offset -1..+1, lines, heading, ...)

Steering units given to decision.py (unchanged rules, per-backend gains in config):
  hough: offset in pixels (STEER_KP / STEER_KD, as calibrated before)
  ufld:  normalised offset -1..+1 (STEER_KP_UFLD / STEER_KD_UFLD / STEER_MAX_UFLD)
"""
from .result import LaneEstimator, LaneResult  # noqa: F401

BACKENDS = ("hough", "ufld")


def make_lane(backend=None, model=None, threads=None):
    import config
    backend = backend or config.LANE_BACKEND
    if backend == "hough":
        from .hough import HoughLane
        return HoughLane()
    if backend == "ufld":
        from .ufld import UfldLane
        return UfldLane(model=model, threads=threads)
    raise ValueError(f"unknown lane backend {backend!r} (use one of {BACKENDS})")


def steer_cfg(backend):
    """Config view for decision.decide(): None = the config module itself (Hough, unchanged);
    for UFLD the steering gains are swapped for the UFLD ones. Nothing else differs."""
    import config
    if backend != "ufld":
        return None
    return _Override(config, STEER_KP=config.STEER_KP_UFLD, STEER_KD=config.STEER_KD_UFLD,
                     STEER_MAX=config.STEER_MAX_UFLD)


class _Override:
    def __init__(self, base, **values):
        self.__dict__["_base"] = base
        self.__dict__["_values"] = values

    def __getattr__(self, name):
        v = self.__dict__["_values"]
        return v[name] if name in v else getattr(self.__dict__["_base"], name)
