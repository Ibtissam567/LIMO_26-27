#!/usr/bin/env python3
"""Measurable recovery preconditions without Gazebo (history + rear sector)."""

import math
import unittest

from limo_control.motion_history import MotionHistory, MotionSample


def rear_clear(ranges, angle_min, angle_inc, half=0.7, need=0.35):
    angle = angle_min
    nearest = None
    for distance in ranges:
        wrapped = math.atan2(math.sin(angle), math.cos(angle))
        if abs(abs(wrapped) - math.pi) <= half or abs(wrapped) >= (math.pi - half):
            if math.isfinite(distance):
                nearest = distance if nearest is None else min(nearest, distance)
        angle += angle_inc
    if nearest is None:
        return True
    return nearest >= need


class RecoveryLogicBench(unittest.TestCase):
    def test_history_provides_safe_anchor(self):
        hist = MotionHistory(50)
        for i in range(20):
            hist.push(
                MotionSample(
                    x=-0.7,
                    y=0.2 + 0.02 * i,
                    yaw=1.57,
                    v=0.05,
                    w=0.0,
                    t=float(i) * 0.1,
                    left_clearance=0.15,
                    right_clearance=0.18,
                    footprint_safe=True,
                    mission="FOLLOW_ROAD",
                )
            )
        # Last samples unsafe at boundary.
        hist.push(
            MotionSample(
                x=-0.55,
                y=0.7,
                yaw=1.4,
                v=0.0,
                w=0.0,
                t=2.1,
                left_clearance=-0.02,
                right_clearance=0.20,
                footprint_safe=False,
                mission="STOP_BOUNDARY",
            )
        )
        safe = hist.last_safe()
        self.assertIsNotNone(safe)
        self.assertTrue(safe.footprint_safe)
        self.assertGreaterEqual(safe.y, 0.2)

    def test_rear_blocked_denies_reverse(self):
        # 360 samples, obstacle at rear.
        n = 360
        angle_inc = 2.0 * math.pi / n
        ranges = [10.0] * n
        # angle ≈ π → index around n/2 if angle_min=-π
        angle_min = -math.pi
        idx = int((math.pi - angle_min) / angle_inc) % n
        ranges[idx] = 0.15
        self.assertFalse(rear_clear(ranges, angle_min, angle_inc, need=0.35))

    def test_rear_open_allows_reverse(self):
        n = 360
        angle_inc = 2.0 * math.pi / n
        ranges = [10.0] * n
        self.assertTrue(rear_clear(ranges, -math.pi, angle_inc, need=0.35))


if __name__ == "__main__":
    unittest.main(verbosity=2)
