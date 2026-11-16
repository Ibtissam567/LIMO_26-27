#!/usr/bin/env python3
"""Map-metric footprint clearance for the LIMO (official track / wheel sizes)."""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

# Official dimensions (limo_driver track_=0.172, URDF wheel_length/radius/wheelbase).
DEFAULT_TRACK = 0.172
DEFAULT_WHEEL_LENGTH = 0.045
DEFAULT_WHEELBASE = 0.20
DEFAULT_WHEEL_RADIUS = 0.045
DEFAULT_SAFETY_MARGIN = 0.05
DEFAULT_LUMA_MAX_ROAD = 90.0
PLANE_X = 4.88
PLANE_Y = 6.2


def robot_half_width(track: float = DEFAULT_TRACK, wheel_length: float = DEFAULT_WHEEL_LENGTH) -> float:
    return 0.5 * track + 0.5 * wheel_length


def robot_length(wheelbase: float = DEFAULT_WHEELBASE, wheel_radius: float = DEFAULT_WHEEL_RADIUS) -> float:
    return wheelbase + 2.0 * wheel_radius


def map_luma(gray: Optional[np.ndarray], x: float, y: float) -> float:
    """Sample map intensity at world (x, y). 255 = bright / off-map."""
    if gray is None:
        return 255.0
    height, width = gray.shape[:2]
    u = (x + PLANE_X / 2.0) / PLANE_X
    v = 1.0 - (y + PLANE_Y / 2.0) / PLANE_Y
    if not (0.0 <= u <= 1.0 and 0.0 <= v <= 1.0):
        return 255.0
    px = min(width - 1, max(0, int(u * (width - 1))))
    py = min(height - 1, max(0, int(v * (height - 1))))
    return float(gray[py, px])


def is_drivable(gray: Optional[np.ndarray], x: float, y: float, luma_max: float = DEFAULT_LUMA_MAX_ROAD) -> bool:
    return map_luma(gray, x, y) <= float(luma_max)


def body_to_world(pose: Tuple[float, float, float], bx: float, by: float) -> Tuple[float, float]:
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + bx * c - by * s, y + bx * s + by * c)


def footprint_sample_points(
    pose: Tuple[float, float, float],
    half_width: float,
    length: float,
) -> list:
    """Corners, mid-sides, and four wheel centres in the world frame."""
    half_length = 0.5 * length
    axle_f = 0.5 * DEFAULT_WHEELBASE
    axle_r = -0.5 * DEFAULT_WHEELBASE
    local = [
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, half_width),
        (-half_length, -half_width),
        (half_length, 0.0),
        (-half_length, 0.0),
        (0.0, half_width),
        (0.0, -half_width),
        (axle_f, half_width),
        (axle_f, -half_width),
        (axle_r, half_width),
        (axle_r, -half_width),
    ]
    return [body_to_world(pose, bx, by) for bx, by in local]


def distance_to_non_road(
    gray: Optional[np.ndarray],
    pose: Tuple[float, float, float],
    side: float,
    half_width: float,
    luma_max: float,
    max_range: float = 0.60,
    step: float = 0.02,
) -> float:
    """
    Distance from robot centre laterally to first non-drivable cell.
    side: +1 = left, -1 = right (robot frame).
    """
    # Start just outside the body half-width so we measure remaining margin.
    lateral = half_width
    while lateral <= max_range:
        wx, wy = body_to_world(pose, 0.05, side * lateral)
        if not is_drivable(gray, wx, wy, luma_max):
            return lateral
        # Also check slightly forward/back (axles).
        for bx in (0.10, -0.10):
            wx, wy = body_to_world(pose, bx, side * lateral)
            if not is_drivable(gray, wx, wy, luma_max):
                return lateral
        lateral += step
    return max_range


def evaluate_pose(
    gray: Optional[np.ndarray],
    pose: Tuple[float, float, float],
    half_width: float,
    length: float,
    safety_margin: float,
    luma_max: float = DEFAULT_LUMA_MAX_ROAD,
) -> dict:
    points = footprint_sample_points(pose, half_width + safety_margin, length + 2.0 * safety_margin)
    points_ok = all(is_drivable(gray, px, py, luma_max) for px, py in points)
    dist_left = distance_to_non_road(gray, pose, +1.0, half_width, luma_max)
    dist_right = distance_to_non_road(gray, pose, -1.0, half_width, luma_max)
    left_clearance = dist_left - half_width - safety_margin
    right_clearance = dist_right - half_width - safety_margin
    footprint_safe = bool(points_ok and left_clearance >= 0.0 and right_clearance >= 0.0)
    return {
        "left_clearance": float(left_clearance),
        "right_clearance": float(right_clearance),
        "footprint_safe": footprint_safe,
        "dist_left": float(dist_left),
        "dist_right": float(dist_right),
    }


def swept_footprint_safe(
    gray: Optional[np.ndarray],
    pose: Tuple[float, float, float],
    goal_xy: Optional[Tuple[float, float]],
    half_width: float,
    length: float,
    safety_margin: float,
    luma_max: float = DEFAULT_LUMA_MAX_ROAD,
    steps: int = 5,
    max_ahead: float = 0.35,
) -> bool:
    """Check current pose and a few poses along the way to goal (short horizon)."""
    current = evaluate_pose(gray, pose, half_width, length, safety_margin, luma_max)
    if not current["footprint_safe"]:
        return False
    if goal_xy is None:
        return True
    x, y, yaw = pose
    gx, gy = goal_xy
    dx, dy = gx - x, gy - y
    dist = math.hypot(dx, dy)
    if dist < 1e-3:
        return True
    ahead = min(dist, max_ahead)
    bearing = math.atan2(dy, dx)
    for index in range(1, steps + 1):
        ratio = index / float(steps)
        px = x + ahead * ratio * math.cos(bearing)
        py = y + ahead * ratio * math.sin(bearing)
        # Blend yaw toward travel direction for outer-wheel sweep.
        pyaw = yaw + 0.5 * math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
        result = evaluate_pose(gray, (px, py, pyaw), half_width, length, safety_margin, luma_max)
        if not result["footprint_safe"]:
            return False
    return True


def compute_footprint_clearance(
    gray: Optional[np.ndarray],
    pose: Optional[Tuple[float, float, float]],
    goal_xy: Optional[Tuple[float, float]] = None,
    track: float = DEFAULT_TRACK,
    wheel_length: float = DEFAULT_WHEEL_LENGTH,
    wheelbase: float = DEFAULT_WHEELBASE,
    wheel_radius: float = DEFAULT_WHEEL_RADIUS,
    safety_margin: float = DEFAULT_SAFETY_MARGIN,
    luma_max: float = DEFAULT_LUMA_MAX_ROAD,
) -> dict:
    half_width = robot_half_width(track, wheel_length)
    length = robot_length(wheelbase, wheel_radius)
    if pose is None or gray is None:
        return {
            "left_clearance": 0.0,
            "right_clearance": 0.0,
            "footprint_safe": False,
            "half_width": half_width,
            "length": length,
        }
    result = evaluate_pose(gray, pose, half_width, length, safety_margin, luma_max)
    if result["footprint_safe"] and goal_xy is not None:
        if not swept_footprint_safe(
            gray, pose, goal_xy, half_width, length, safety_margin, luma_max
        ):
            result["footprint_safe"] = False
            # Shrink the tighter side slightly to signal upcoming breach.
            if result["left_clearance"] < result["right_clearance"]:
                result["left_clearance"] = min(result["left_clearance"], -0.01)
            else:
                result["right_clearance"] = min(result["right_clearance"], -0.01)
    result["half_width"] = half_width
    result["length"] = length
    return result
