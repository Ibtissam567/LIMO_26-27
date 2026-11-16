#!/usr/bin/env python3
"""Short-horizon candidate goals scored by footprint clearance (no MPC)."""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

from limo_perception.footprint_safety import (
    DEFAULT_LUMA_MAX_ROAD,
    DEFAULT_SAFETY_MARGIN,
    DEFAULT_TRACK,
    DEFAULT_WHEEL_LENGTH,
    DEFAULT_WHEELBASE,
    DEFAULT_WHEEL_RADIUS,
    compute_footprint_clearance,
    evaluate_pose,
    robot_half_width,
    robot_length,
)

Pose2 = Tuple[float, float, float]
Goal2 = Tuple[float, float, float]


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def build_candidates(
    x: float,
    y: float,
    yaw: float,
    base_goal: Goal2,
    lookahead: float,
    lateral_nudge: float = 0.12,
    short_scale: float = 0.55,
) -> List[Dict]:
    """Straight, left, right, and reduced-lookahead variants around a base goal."""
    gx, gy, gyaw = base_goal
    dx = gx - x
    dy = gy - y
    dist = math.hypot(dx, dy)
    bearing = math.atan2(dy, dx) if dist > 1e-6 else yaw
    # Lateral unit in world frame (robot left).
    left_x = -math.sin(yaw)
    left_y = math.cos(yaw)

    candidates = [
        {"name": "straight", "goal": (gx, gy, gyaw), "speed_scale": 1.0},
        {
            "name": "left",
            "goal": (gx + lateral_nudge * left_x, gy + lateral_nudge * left_y, gyaw),
            "speed_scale": 0.85,
        },
        {
            "name": "right",
            "goal": (gx - lateral_nudge * left_x, gy - lateral_nudge * left_y, gyaw),
            "speed_scale": 0.85,
        },
    ]
    short = max(0.12, min(dist, lookahead) * short_scale)
    sx = x + short * math.cos(bearing)
    sy = y + short * math.sin(bearing)
    candidates.append(
        {"name": "slow_short", "goal": (sx, sy, bearing), "speed_scale": 0.55}
    )
    return candidates


def _progress_to_route(
    goal_xy: Tuple[float, float],
    route: Sequence[Tuple[float, float]],
    mission_xy: Tuple[float, float],
) -> float:
    if route:
        nearest = min(math.hypot(goal_xy[0] - px, goal_xy[1] - py) for px, py in route)
        return max(0.0, 1.0 - nearest / 0.6)
    return max(0.0, 1.0 - math.hypot(goal_xy[0] - mission_xy[0], goal_xy[1] - mission_xy[1]) / 3.0)


def score_candidate(
    gray,
    pose: Pose2,
    candidate: Dict,
    route: Sequence[Tuple[float, float]],
    mission_xy: Tuple[float, float],
    safety_margin: float = DEFAULT_SAFETY_MARGIN,
    track: float = DEFAULT_TRACK,
    wheel_length: float = DEFAULT_WHEEL_LENGTH,
    wheelbase: float = DEFAULT_WHEELBASE,
    wheel_radius: float = DEFAULT_WHEEL_RADIUS,
    luma_max: float = DEFAULT_LUMA_MAX_ROAD,
    min_clearance: float = 0.0,
) -> Dict:
    """Higher score is better. Unsafe footprint → rejected."""
    gx, gy, gyaw = candidate["goal"]
    clearance = compute_footprint_clearance(
        gray,
        pose,
        goal_xy=(gx, gy),
        track=track,
        wheel_length=wheel_length,
        wheelbase=wheelbase,
        wheel_radius=wheel_radius,
        safety_margin=safety_margin,
        luma_max=luma_max,
    )
    half_width = robot_half_width(track, wheel_length)
    length = robot_length(wheelbase, wheel_radius)
    goal_eval = evaluate_pose(
        gray, (gx, gy, gyaw), half_width, length, safety_margin, luma_max
    )
    left_c = float(clearance["left_clearance"])
    right_c = float(clearance["right_clearance"])
    min_c = min(left_c, right_c, float(goal_eval["left_clearance"]), float(goal_eval["right_clearance"]))
    safe = bool(clearance["footprint_safe"] and goal_eval["footprint_safe"] and min_c >= min_clearance)
    if not safe:
        return {
            "name": candidate["name"],
            "goal": candidate["goal"],
            "speed_scale": float(candidate.get("speed_scale", 1.0)),
            "score": float("-inf"),
            "safe": False,
            "min_clearance": min_c,
        }

    progress = _progress_to_route((gx, gy), route, mission_xy)
    # Prefer balanced clearance and smoother heading change.
    heading_pen = abs(_wrap(gyaw - pose[2]))
    balance = 1.0 - min(1.0, abs(left_c - right_c) / 0.25)
    score = (
        3.0 * min_c
        + 1.2 * progress
        + 0.4 * balance
        + 0.3 * float(candidate.get("speed_scale", 1.0))
        - 0.25 * heading_pen
    )
    return {
        "name": candidate["name"],
        "goal": candidate["goal"],
        "speed_scale": float(candidate.get("speed_scale", 1.0)),
        "score": float(score),
        "safe": True,
        "min_clearance": min_c,
    }


def select_best_candidate(
    gray,
    pose: Pose2,
    base_goal: Goal2,
    lookahead: float,
    route: Sequence[Tuple[float, float]],
    mission_xy: Tuple[float, float],
    lateral_nudge: float = 0.12,
    **score_kwargs,
) -> Optional[Dict]:
    candidates = build_candidates(
        pose[0], pose[1], pose[2], base_goal, lookahead, lateral_nudge=lateral_nudge
    )
    scored = [score_candidate(gray, pose, item, route, mission_xy, **score_kwargs) for item in candidates]
    safe = [item for item in scored if item["safe"]]
    if not safe:
        return None
    return max(safe, key=lambda item: item["score"])
