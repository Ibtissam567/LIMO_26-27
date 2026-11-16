#!/usr/bin/env python3
"""Unit tests for map-metric footprint clearance (no Gazebo)."""

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from limo_perception.footprint_safety import (
    compute_footprint_clearance,
    robot_half_width,
    evaluate_pose,
)


def make_corridor_map(width_m=0.50, world_half=2.44):
    """Synthetic greyscale map: black road strip along x=0, white elsewhere."""
    # Map uses PLANE_X=4.88, PLANE_Y=6.2 → image coords via same convention as footprint_safety.
    h, w = 310, 244
    gray = np.full((h, w), 220, dtype=np.uint8)
    # Paint a vertical black corridor around x=0 of given width.
    for py in range(h):
        for px in range(w):
            u = px / (w - 1)
            v = py / (h - 1)
            x = u * 4.88 - 2.44
            y = (1.0 - v) * 6.2 - 3.1
            if abs(x) <= 0.5 * width_m:
                gray[py, px] = 20
    return gray


class FootprintSafetyTest(unittest.TestCase):
    def test_half_width_from_official_dims(self):
        # track 0.172, wheel_length 0.045 → 0.1085
        self.assertAlmostEqual(robot_half_width(0.172, 0.045), 0.1085, places=4)

    def test_centred_pose_is_safe(self):
        gray = make_corridor_map(width_m=0.55)
        pose = (0.0, 0.0, 1.5708)  # facing +Y on centre line
        result = compute_footprint_clearance(gray, pose, goal_xy=None, safety_margin=0.05)
        self.assertTrue(result["footprint_safe"])
        self.assertGreaterEqual(result["left_clearance"], 0.0)
        self.assertGreaterEqual(result["right_clearance"], 0.0)

    def test_shifted_toward_edge_is_unsafe(self):
        gray = make_corridor_map(width_m=0.40)
        # Shift right so outer wheel / margin hits white.
        pose = (0.12, 0.0, 1.5708)
        result = evaluate_pose(
            gray,
            pose,
            half_width=0.1085,
            length=0.29,
            safety_margin=0.05,
        )
        self.assertFalse(result["footprint_safe"])
        self.assertLess(min(result["left_clearance"], result["right_clearance"]), 0.0)

    def test_missing_map_is_unsafe(self):
        result = compute_footprint_clearance(None, (0.0, 0.0, 0.0))
        self.assertFalse(result["footprint_safe"])


if __name__ == "__main__":
    unittest.main()
