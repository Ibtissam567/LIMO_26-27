#!/usr/bin/env python3
"""Measurable before/after: naive straight goal vs predictive pick on the real track map."""

import math
import os
import sys

import cv2
from ament_index_python.packages import get_package_share_directory

from limo_perception.footprint_safety import compute_footprint_clearance
from limo_planning.candidate_trajectories import select_best_candidate


def load_map():
    share = get_package_share_directory("qcar_large_track")
    path = os.path.join(
        share, "models/qcar_large_track/materials/textures/SDCS_MapLayout.png"
    )
    gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if gray is None:
        raise SystemExit(f"map missing: {path}")
    return gray


def min_clearance(gray, pose, goal_xy):
    result = compute_footprint_clearance(gray, pose, goal_xy=goal_xy)
    return min(result["left_clearance"], result["right_clearance"]), bool(result["footprint_safe"])


def main():
    gray = load_map()
    # Poses where current footprint is safe but a 0.40 m straight goal is not.
    cases = [
        ((-2.040, -1.200, 1.571), 0.40),
        ((-2.040, 0.240, 4.712), 0.40),
        ((-2.040, 0.720, 0.000), 0.40),
        ((-2.040, 1.680, 1.571), 0.40),
        ((-1.920, -1.200, 0.785), 0.40),
        ((-1.920, -1.200, 1.571), 0.40),
        ((-1.920, -0.600, 5.498), 0.40),
        ((-1.920, -0.240, 1.571), 0.40),
    ]
    before_safe = 0
    after_safe = 0
    rescued = 0
    print("pose_x pose_y yaw | before_safe | after_safe pick after_clear")
    for (x, y, yaw), lookahead in cases:
        gx = x + lookahead * math.cos(yaw)
        gy = y + lookahead * math.sin(yaw)
        _, b_safe = min_clearance(gray, (x, y, yaw), (gx, gy))
        picked = select_best_candidate(
            gray,
            (x, y, yaw),
            (gx, gy, yaw),
            lookahead,
            route=[],
            mission_xy=(-0.725, 2.413),
            lateral_nudge=0.14,
        )
        a_safe = picked is not None
        before_safe += int(b_safe)
        after_safe += int(a_safe)
        if (not b_safe) and a_safe:
            rescued += 1
        name = picked["name"] if picked else "none"
        a_clear = picked["min_clearance"] if picked else float("nan")
        print(
            f"{x:7.3f} {y:7.3f} {yaw:5.3f} | {str(b_safe):5} | {str(a_safe):5} {name:10} {a_clear:7.3f}"
        )

    print("---")
    print(f"cases={len(cases)}")
    print(f"BEFORE safe_goals={before_safe}/{len(cases)}")
    print(f"AFTER  safe_goals={after_safe}/{len(cases)}")
    print(f"rescued_from_unsafe_straight={rescued}/{len(cases)}")
    if after_safe <= before_safe:
        print("REGRESSION: predictive did not increase safe goals")
        sys.exit(2)
    if rescued < 1:
        print("REGRESSION: no rescue cases")
        sys.exit(3)
    print("BENCH_OK")


if __name__ == "__main__":
    main()
