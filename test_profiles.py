#!/usr/bin/env python3
"""Runtime profiles (config.RUNTIME_PROFILES, main_demo.py --profile) and the INT8 UFLD model entry.

  python test_profiles.py
"""
import hashlib
import os
import unittest

import numpy as np

import config
import main_demo


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


class TestProfiles(unittest.TestCase):
    def test_profiles_defined(self):
        self.assertEqual(config.RUNTIME_PROFILES["ufld_solo"]["ufld_model"], "v2_same")       # FP32 alone
        self.assertFalse(config.RUNTIME_PROFILES["ufld_solo"]["yolo"])
        self.assertEqual(config.RUNTIME_PROFILES["ufld_yolo"]["ufld_model"], "v2_same_int8")  # INT8 with YOLO
        self.assertTrue(config.RUNTIME_PROFILES["ufld_yolo"]["yolo"])
        self.assertIn(config.DEMO_PROFILE, config.RUNTIME_PROFILES)
        for p in config.RUNTIME_PROFILES.values():
            self.assertIn(p["ufld_model"], config.UFLD_MODELS)

    def test_apply_profile(self):
        a = main_demo.parse_args(["--web", "--profile", "ufld_yolo"])
        self.assertEqual((a.lane, a.ufld_model, a.ufld_threads, a.yolo_threads, a.yolo_every, a.no_yolo),
                         ("ufld", "v2_same_int8", 3, 1, 3, False))
        a = main_demo.parse_args(["--web", "--profile", "ufld_solo"])
        self.assertEqual((a.lane, a.ufld_model, a.ufld_threads, a.no_yolo), ("ufld", "v2_same", 4, True))

    def test_explicit_flags_win(self):
        a = main_demo.parse_args(["--web", "--profile", "ufld_yolo", "--ufld-threads", "2", "--ufld-model", "v2_same"])
        self.assertEqual((a.ufld_threads, a.ufld_model, a.yolo_every), (2, "v2_same", 3))

    def test_no_profile_unchanged(self):
        a = main_demo.parse_args(["--web"])
        self.assertEqual((a.lane, a.ufld_model, a.ufld_threads, a.yolo_every, a.no_yolo), (None, None, None, None, False))

    def test_int8_checksum_recorded_and_matches(self):
        sums = {}
        with open(config.UFLD_SHA256) as f:
            for line in f:
                if line.strip():
                    d, n = line.split()
                    sums[n.lstrip("*")] = d
        path = config.UFLD_MODELS["v2_same_int8"]
        name = os.path.basename(path)
        self.assertEqual(sums[name], "d50f3deae53ea9f944b491d57851ce7d8beff8dc82bf74598fbe7b0fb1153e4a")
        self.assertEqual(sha256(path), sums[name])

    def test_int8_model_runs(self):
        from lanes import make_lane
        lane = make_lane("ufld", model="v2_same_int8", threads=3)
        r = lane.process(np.zeros((config.CAM_HEIGHT, config.CAM_WIDTH, 3), np.uint8))
        self.assertEqual(lane.source, "ufld:v2_same_int8")
        self.assertIsNotNone(r)


if __name__ == "__main__":
    unittest.main(verbosity=2)
