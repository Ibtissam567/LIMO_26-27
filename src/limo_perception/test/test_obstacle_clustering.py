#!/usr/bin/env python3
"""Unit tests for adaptive LiDAR clustering (no Gazebo)."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from limo_perception.obstacle_detector import adaptive_gap, cluster_points


class AdaptiveClusteringTest(unittest.TestCase):
    def test_gap_grows_with_range(self):
        near = adaptive_gap(0.5, gap_base=0.12, gap_k=2.5, angle_increment=0.01)
        far = adaptive_gap(2.0, gap_base=0.12, gap_k=2.5, angle_increment=0.01)
        self.assertGreater(far, near)
        self.assertAlmostEqual(near, 0.12 + 2.5 * 0.5 * 0.01, places=5)

    def test_far_points_stay_one_cluster(self):
        # Two returns from one object at ~2 m, angularly adjacent.
        angle_inc = 0.02
        r = 2.0
        p0 = (r * math.cos(0.0), r * math.sin(0.0), r)
        p1 = (r * math.cos(angle_inc), r * math.sin(angle_inc), r)
        clusters = cluster_points(
            [p0, p1],
            gap_base=0.12,
            gap_k=2.5,
            angle_increment=angle_inc,
            min_points=2,
        )
        self.assertEqual(len(clusters), 1)

    def test_min_cluster_points_filters_noise(self):
        points = [(1.0, 0.0, 1.0), (1.05, 0.0, 1.05)]
        clusters = cluster_points(
            points,
            gap_base=0.2,
            gap_k=0.0,
            angle_increment=0.01,
            min_points=3,
        )
        self.assertEqual(clusters, [])


if __name__ == "__main__":
    unittest.main()
