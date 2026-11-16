#!/usr/bin/env python3
"""Bounded rolling buffer of recent robot states for recovery."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional


@dataclass
class MotionSample:
    x: float
    y: float
    yaw: float
    v: float
    w: float
    t: float
    left_clearance: float
    right_clearance: float
    footprint_safe: bool
    mission: str


class MotionHistory:
    def __init__(self, maxlen: int = 50):
        self._buf: Deque[MotionSample] = deque(maxlen=max(1, int(maxlen)))

    def __len__(self) -> int:
        return len(self._buf)

    def push(self, sample: MotionSample) -> None:
        self._buf.append(sample)

    def latest(self) -> Optional[MotionSample]:
        return self._buf[-1] if self._buf else None

    def last_safe(self, min_clearance: float = 0.0) -> Optional[MotionSample]:
        for sample in reversed(self._buf):
            if (
                sample.footprint_safe
                and sample.left_clearance >= min_clearance
                and sample.right_clearance >= min_clearance
            ):
                return sample
        return None

    def progress_since(self, older: MotionSample, newer: MotionSample) -> float:
        import math

        return math.hypot(newer.x - older.x, newer.y - older.y)

    def samples(self):
        return list(self._buf)
