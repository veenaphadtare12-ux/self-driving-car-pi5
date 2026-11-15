#!/usr/bin/env python3
"""End-to-end check of a RUNNING demo page (main_demo.py --web ...), driven over HTTP like a browser.
DRY RUN only: refuses to run if the page says motors are live.

  python check_web_e2e.py [--base http://127.0.0.1:8001] [--seconds 20] [--out logs/web_e2e]

  1. page load time; command response times (START/STOP/E-STOP/phase)
  2. control rate / lane / detection fps with the page closed, stream open, stream + RECORD
  3. phases P1, P2, P7: START, wait, save a stream frame + state/reason/ignored
  4. second browser: view-only (claim 409, START 403), but E-STOP works
  5. heartbeat: "close" the controlling page while driving -> time to STOP
"""
import argparse
import json
import os
import statistics
import threading
import time
import urllib.error
import urllib.request


class Browser:
    def __init__(self, base):
        self.base, self.token = base, None
        self.hb_stop = threading.Event()
        self.hb_thread = None

    def post(self, path, body=None):
        b = dict(body or {})
        b["token"] = self.token
        req = urllib.request.Request(self.base + path, json.dumps(b).encode(), {"Content-Type": "application/json"})
        t0 = time.perf_counter()
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                code, obj = r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            code, obj = e.code, json.loads(e.read())
        return code, obj, time.perf_counter() - t0

    def status(self):
        with urllib.request.urlopen(f"{self.base}/status?token={self.token or ''}", timeout=5) as r:
            return json.loads(r.read())

    def claim(self, force=False):
        code, obj, _ = self.post("/claim", {"force": force})
        if code == 200:
            self.token = obj["token"]
        return code

    def start_heartbeat(self):
        self.hb_stop.clear()

        def run():
            while not self.hb_stop.wait(0.5):
                try:
                    self.post("/heartbeat")
                except Exception:                                # noqa: BLE001
                    pass
        self.hb_thread = threading.Thread(target=run, daemon=True)
        self.hb_thread.start()

    def stop_heartbeat(self):
        self.hb_stop.set()
        if self.hb_thread:
            self.hb_thread.join()
        return time.monotonic()


class Stream:
    def __init__(self, base):
        self.r = urllib.request.urlopen(base + "/stream", timeout=5)
        self.stop = threading.Event()
        self.last_jpeg, self.frames = None, 0
        self.t = threading.Thread(target=self.run, daemon=True)
        self.t.start()

    def run(self):
        buf = b""
        try:
            while not self.stop.is_set():
                buf += self.r.read(16384)
                while True:
                    i = buf.find(b"\xff\xd8")
                    j = buf.find(b"\xff\xd9", i + 2)
                    if i < 0 or j < 0:
                        break
                    self.last_jpeg, buf = buf[i:j + 2], buf[j + 2:]
                    self.frames += 1
        except Exception:                                        # noqa: BLE001
            pass

    def close(self):
        self.stop.set()
        self.r.close()


def sample(b, seconds):
    rows = []
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        s = b.status()
        rows.append((s["ctrl_hz"], s["lane_fps"], s["det_fps"], s["view_fps"]))
        time.sleep(0.5)
    rows = rows[4:] or rows                                      # skip the 3 s meter warm-up
    m = lambda k: round(statistics.mean(r[k] or 0 for r in rows), 2)
    return {"ctrl_hz": m(0), "ctrl_hz_min": min(r[0] or 0 for r in rows), "lane_fps": m(1), "det_fps": m(2),
            "view_fps": m(3)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", default="http://127.0.0.1:8001")
    ap.add_argument("--seconds", type=float, default=20)
    ap.add_argument("--out", default="logs/web_e2e")
    ap.add_argument("--phase-wait", type=float, default=8)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    res = {}
    A, B = Browser(a.base), Browser(a.base)

    t0 = time.perf_counter()
    with urllib.request.urlopen(a.base + "/", timeout=5) as r:
        page = r.read()
    res["page_load_s"] = round(time.perf_counter() - t0, 3)
    s = A.status()
    assert s["dry_run"], "motors are LIVE - this check only runs in DRY RUN"
    res["initial"] = {"state": s["state"], "dry_run": s["dry_run"], "dry_locked": s["dry_locked"], "lane": s["lane"]}
    print("initial:", res["initial"], "page", len(page), "bytes in", res["page_load_s"], "s")

    # 2. rates
    res["rates_page_closed"] = sample(A, a.seconds)
    print("page closed  :", res["rates_page_closed"])
    st = Stream(a.base)
    res["rates_stream_open"] = sample(A, a.seconds)
    print("stream open  :", res["rates_stream_open"])
    assert A.claim() == 200
    A.start_heartbeat()
    A.post("/record", {"on": True})
    res["rates_stream_record"] = sample(A, a.seconds)
    rec = A.status()
    A.post("/record", {"on": False})
    res["recording"] = {"video": rec["rec_path"], "csv": rec["rec_csv"]}
    print("stream+record:", res["rates_stream_record"], res["recording"])

    # 1. command times + 3. phases
    times = []
    res["phases"] = {}
    for ph in ("P1", "P2", "P7"):
        code, obj, dt = A.post("/phase", {"phase": ph})
        times.append(("phase", dt))
        code, obj, dt = A.post("/cmd", {"action": "start"})
        times.append(("start", dt))
        states = {}
        t_end = time.monotonic() + a.phase_wait
        last = None
        while time.monotonic() < t_end:
            last = A.status()
            states[last["state"]] = states.get(last["state"], 0) + 1
            time.sleep(0.25)
        if st.last_jpeg:
            with open(os.path.join(a.out, f"{ph}.jpg"), "wb") as f:
                f.write(st.last_jpeg)
        res["phases"][ph] = {"states_seen": states, "last_state": last["state"], "last_reason": last["reason"],
                             "ignored": last["ignored"]}
        print(f"{ph}: states {states} | last {last['state']}: {last['reason'][:70]} | ignored {last['ignored']}")
        code, obj, dt = A.post("/cmd", {"action": "stop"})
        times.append(("stop", dt))

    # 4. second browser
    c1 = B.claim()
    A.post("/cmd", {"action": "start"})
    c2, _, _ = B.post("/cmd", {"action": "stop"})
    time.sleep(0.3)
    running_after_b_stop = A.status()["running"]
    c3, _, dt = B.post("/cmd", {"action": "estop"})
    times.append(("estop", dt))
    time.sleep(0.3)
    s = A.status()
    res["second_browser"] = {"claim": c1, "stop_from_viewer": c2, "running_after_viewer_stop": running_after_b_stop,
                             "estop_from_viewer": c3, "state_after": s["state"], "dry_run_after": s["dry_run"]}
    print("second browser:", res["second_browser"])

    # 5. heartbeat: START, then the controlling page "closes"
    A.post("/cmd", {"action": "start"})
    time.sleep(1.5)
    assert A.status()["running"]
    t_close = A.stop_heartbeat()
    while A.status()["running"] and time.monotonic() - t_close < 5:
        time.sleep(0.02)
    s = A.status()
    res["heartbeat"] = {"stop_after_s": round(time.monotonic() - t_close, 2), "message": s["message"]}
    print("heartbeat:", res["heartbeat"])
    st.close()
    res["command_ms"] = {k: round(max(dt for n, dt in times if n == k) * 1000, 1) for k in {n for n, _ in times}}
    res["command_max_s"] = round(max(dt for _, dt in times), 3)
    print("command response max ms:", res["command_ms"])
    with open(os.path.join(a.out, "result.json"), "w") as f:
        json.dump(res, f, indent=1)


if __name__ == "__main__":
    main()
