#!/usr/bin/env python3
"""Unit tests for STOP vs red-light blob rules (no Gazebo)."""

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from limo_perception.sign_detector import blobs_overlap, detect_signs, confidence_with_streak


class SignRulesTest(unittest.TestCase):
    def test_upper_red_disk_is_light_not_stop(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        # Compact red disk in the upper ROI (traffic light).
        cv2.circle(image, (160, 40), 12, (0, 0, 220), -1)
        found = detect_signs(image, min_stop_pixels=250, min_light_pixels=40)
        types = [item["type"] for item in found]
        self.assertIn("FEU_ROUGE", types)
        self.assertNotIn("STOP", types)

    def test_midframe_red_plate_is_stop(self):
        image = np.zeros((240, 320, 3), dtype=np.uint8)
        # Large roughly-square red plate in mid frame.
        cv2.rectangle(image, (120, 80), (200, 170), (0, 0, 220), -1)
        found = detect_signs(image, min_stop_pixels=250, min_light_pixels=40)
        types = [item["type"] for item in found]
        self.assertIn("STOP", types)

    def test_overlap_helper(self):
        a = (10, 10, 40, 40, 1600, 1.0)
        b = (20, 20, 40, 40, 1600, 1.0)
        c = (200, 200, 20, 20, 400, 1.0)
        self.assertTrue(blobs_overlap(a, b))
        self.assertFalse(blobs_overlap(a, c))

    def test_streak_raises_confidence(self):
        low = confidence_with_streak(0.5, streak=1, needed=3)
        high = confidence_with_streak(0.5, streak=3, needed=3)
        self.assertGreater(high, low)


if __name__ == "__main__":
    unittest.main()
