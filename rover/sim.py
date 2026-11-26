"""Simulated rover + IMU for dry runs and tests (no hardware).

Turns land within about +-10% of the command (like skid turns on the floor),
lanes pick up a little heading drift, and the true pose is kept so plans can
be checked against what odometry believes.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from rover.config import RoverConfig, load_config


@dataclass
class _Result:
    kind: str
    reason: str
    left_ticks: int
    right_ticks: int

    @property
    def reached_target(self) -> bool:
        return self.reason == "TARGET"


class SimRover:
    def __init__(self, config: RoverConfig | None = None, seed: int = 0,
                 turn_scale: float = 1.0, drift_deg_per_m: float = 2.0):
        self.config = config or load_config()
        self.rng = random.Random(seed)
        self.turn_scale = turn_scale
        self.drift = drift_deg_per_m
        self.yaw = 0.0                 # also serves as the IMU
        self.x = self.y = 0.0          # true position, mm
        self.left = self.right = 0
        self.ms = 0
        self.moves = 0

    # Rover API used by the survey
    def encoders(self):
        return self.ms, self.left, self.right

    def battery_mv(self):
        return 12300

    def stop(self):
        pass

    def drive_mm(self, mm, speed=None):
        self.moves += 1
        ticks = round(mm * self.config.ticks_per_mm)
        self.left += ticks
        self.right += ticks
        travelled = mm * self.rng.uniform(0.98, 1.02)
        # drift spread across the move
        d_yaw = self.rng.uniform(-1, 1) * self.drift * abs(mm) / 1000
        mid = math.radians(self.yaw + d_yaw / 2)
        self.x += travelled * math.sin(mid)
        self.y += travelled * math.cos(mid)
        self.yaw += d_yaw
        self.ms += int(abs(mm) / 0.28)
        return _Result("D", "TARGET", ticks, ticks)

    def turn_deg(self, degrees, speed=None):
        self.moves += 1
        ticks = round(abs(degrees) * self.config.ticks_per_degree) * (1 if degrees > 0 else -1)
        self.left += ticks
        self.right -= ticks
        if abs(degrees) >= 2.5:
            self.yaw += degrees * self.turn_scale * self.rng.uniform(0.9, 1.1) + self.rng.uniform(-1, 1)
        self.ms += int(abs(degrees) / 0.12)
        return _Result("R", "TARGET", ticks, -ticks)
