"""Turning and straight driving checked by the IMU heading.

Skid-steer turns scatter about +-10% by encoder counts alone. So a turn is
made with the encoder estimate first, then the IMU measures what actually
happened and small corrective turns close the gap. The ratio of measured to
commanded degrees is remembered, so corrections (and later turns) aim better.

Targets are absolute yaw (unwrapped, clockwise +, from the IMU at start), so
heading drift picked up while driving a lane is removed at the next turn.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

TOLERANCE_DEG = 1.5
MAX_CORRECTIONS = 6
SETTLE_S = 0.25           # let the chassis stop rocking before reading the heading
MIN_COMMAND_DEG = 3.0     # smaller skid turns tend not to move at all
RATIO_LIMITS = (0.6, 1.6)
GIVE_UP_DEG = 5.0         # raise if a turn still ends further off than this


class HeadingError(RuntimeError):
    pass


@dataclass
class TurnReport:
    target: float
    final: float
    steps: list[tuple[float, float]] = field(default_factory=list)  # (commanded, measured) deg

    @property
    def error(self) -> float:
        return self.target - self.final


class HeadingControl:
    """rover: rover.Rover (turn_deg, drive_mm); imu: anything with a `.yaw`."""

    def __init__(self, rover, imu, tolerance: float = TOLERANCE_DEG):
        self.rover = rover
        self.imu = imu
        self.tolerance = tolerance
        self.ratio = 1.0          # measured / commanded, learned from turns

    def settled_yaw(self) -> float:
        time.sleep(SETTLE_S)
        return self.imu.yaw

    def turn_by(self, degrees: float, speed: int | None = None) -> TurnReport:
        return self.turn_to(self.settled_yaw() + degrees, speed)

    def turn_to(self, target: float, speed: int | None = None) -> TurnReport:
        """Rotate in place until the yaw is within tolerance of `target`."""
        yaw = self.settled_yaw()
        report = TurnReport(target, yaw)
        stuck = 0
        min_command = MIN_COMMAND_DEG
        prev_err = None
        for _ in range(1 + MAX_CORRECTIONS):
            err = target - yaw
            if abs(err) <= self.tolerance:
                break
            if prev_err is not None and err * prev_err < 0:
                # Overshot: the smallest nudge is too big, so try smaller ones.
                min_command = max(1.0, min_command * 0.7)
            prev_err = err
            command = err / self.ratio
            if abs(command) < min_command:
                command = min_command if command > 0 else -min_command
            result = self.rover.turn_deg(command, speed)
            if result.reason not in ("TARGET", "STALL"):
                raise HeadingError(f"turn stopped: {result.reason}")
            new_yaw = self.settled_yaw()
            moved = new_yaw - yaw
            report.steps.append((round(command, 2), round(moved, 2)))
            if abs(moved) < 0.5:
                # Too small to break the tracks loose: nudge harder next time.
                stuck += 1
                min_command = max(min_command, abs(command)) * 1.5
                if stuck >= 3:
                    raise HeadingError(f"not turning (target {target:.1f}, yaw {new_yaw:.1f})")
            else:
                stuck = 0
                # Learn only from turns big enough to measure well, and blend
                # so one odd turn doesn't swing it.
                if abs(command) >= 10 and moved * command > 0:
                    r = min(max(moved / command, RATIO_LIMITS[0]), RATIO_LIMITS[1])
                    self.ratio = 0.5 * self.ratio + 0.5 * r
            yaw = new_yaw
        report.final = yaw
        # A few degrees left over is fine: lanes re-square and the next turn
        # aims at an absolute heading, so it doesn't add up. More means trouble.
        if abs(target - yaw) > GIVE_UP_DEG:
            raise HeadingError(f"turn ended {target - yaw:+.1f} deg off target after {len(report.steps)} moves")
        return report

    def drive_straight(self, mm: float, heading: float, segment_mm: float = 500,
                       speed: int | None = None) -> list[float]:
        """Drive `mm` (negative = backward) in segments, squaring up to
        `heading` before each one. Returns the heading error after each segment."""
        errors = []
        left = mm
        while abs(left) > 1:
            if abs(self.settled_yaw() - heading) > self.tolerance:
                self.turn_to(heading)
            step = max(-segment_mm, min(segment_mm, left))
            result = self.rover.drive_mm(step, speed)
            if not result.reached_target:
                raise HeadingError(f"drive stopped: {result.reason}")
            left -= step
            errors.append(round(self.settled_yaw() - heading, 2))
        return errors
