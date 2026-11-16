#!/usr/bin/env python3
"""Unit tests for motion history buffer."""

import unittest

from limo_control.motion_history import MotionHistory, MotionSample


def _sample(x, y, safe=True, left=0.2, right=0.2, t=0.0):
    return MotionSample(
        x=x,
        y=y,
        yaw=0.0,
        v=0.0,
        w=0.0,
        t=t,
        left_clearance=left,
        right_clearance=right,
        footprint_safe=safe,
        mission="FOLLOW_ROAD",
    )


class MotionHistoryTests(unittest.TestCase):
    def test_bounded(self):
        hist = MotionHistory(maxlen=3)
        for i in range(5):
            hist.push(_sample(float(i), 0.0, t=float(i)))
        self.assertEqual(len(hist), 3)
        self.assertEqual(hist.latest().x, 4.0)

    def test_last_safe_skips_unsafe(self):
        hist = MotionHistory(maxlen=10)
        hist.push(_sample(0.0, 0.0, safe=True, left=0.2, right=0.2))
        hist.push(_sample(0.1, 0.0, safe=False, left=-0.05, right=0.1))
        safe = hist.last_safe(min_clearance=0.0)
        self.assertIsNotNone(safe)
        self.assertEqual(safe.x, 0.0)


if __name__ == "__main__":
    unittest.main()
