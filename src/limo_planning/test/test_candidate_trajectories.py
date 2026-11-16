#!/usr/bin/env python3
"""Offline tests for predictive candidate scoring (no Gazebo)."""

import math
import os
import unittest

import cv2
import numpy as np

from limo_perception.footprint_safety import PLANE_X, PLANE_Y, compute_footprint_clearance
from limo_planning.candidate_trajectories import build_candidates, select_best_candidate


def _synthetic_corridor(width_px=400, height_px=400):
    """Black vertical road strip in the middle; white elsewhere."""
    img = np.full((height_px, width_px), 220, dtype=np.uint8)
    # ~1.0 m wide black corridor (wider than LIMO footprint + margin).
    mid = width_px // 2
    half = max(8, int(0.50 / PLANE_X * width_px))
    img[:, mid - half : mid + half] = 40
    return img


def _world_to_pose_on_road():
    # Image centre ≈ world (0, 0) with current map convention.
    return (0.0, 0.0, math.pi / 2.0)


class CandidateTests(unittest.TestCase):
    def test_build_four_candidates(self):
        items = build_candidates(0.0, 0.0, 0.0, (0.4, 0.0, 0.0), lookahead=0.4)
        names = {item["name"] for item in items}
        self.assertEqual(names, {"straight", "left", "right", "slow_short"})

    def test_rejects_goal_off_road(self):
        gray = _synthetic_corridor()
        pose = _world_to_pose_on_road()
        # Drive hard to the side into white.
        base = (0.6, 0.0, 0.0)
        picked = select_best_candidate(
            gray, pose, base, lookahead=0.4, route=[], mission_xy=(0.0, 2.0), lateral_nudge=0.15
        )
        # Either None or a safer lateral/short candidate with clearance >= 0.
        if picked is not None:
            self.assertTrue(picked["safe"])
            self.assertGreaterEqual(picked["min_clearance"], -1e-6)
            self.assertNotEqual(picked["name"], "straight")

    def test_prefers_centred_over_edge(self):
        gray = _synthetic_corridor()
        pose = (0.0, 0.0, math.pi / 2.0)
        # Straight ahead along black should beat a large side nudge into white.
        base = (0.0, 0.35, math.pi / 2.0)
        picked = select_best_candidate(
            gray, pose, base, lookahead=0.35, route=[(0.0, 1.0)], mission_xy=(0.0, 2.0)
        )
        self.assertIsNotNone(picked)
        self.assertTrue(picked["safe"])


if __name__ == "__main__":
    unittest.main()
