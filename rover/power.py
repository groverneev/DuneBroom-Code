"""Battery checks: the Pi's PiPower 5 pack and the motor pack.

Pi pack: read through the `pipower5` CLI (its own service owns the HAT and
shuts the Pi down at 10 %). Motor pack (3S Li-ion, 11.1 V nominal): voltage
from the motor driver via the Uno. Thresholds live in config/rover.toml
[power]. A survey should refuse to start below `*_start` and stop and save
below `*_stop`; PowerGuard polls in the background and sets `.low`.
"""

from __future__ import annotations

import re
import subprocess
import threading
import tomllib
from dataclasses import dataclass
from pathlib import Path

from rover.config import DEFAULT_CONFIG_PATH


@dataclass(frozen=True)
class PowerLimits:
    pi_start_percent: int
    pi_stop_percent: int
    motor_start_mv: int
    motor_stop_mv: int


@dataclass(frozen=True)
class PowerStatus:
    pi_percent: int | None
    pi_plugged_in: bool | None
    motor_mv: int | None

    def problems(self, limits: PowerLimits, starting: bool) -> list[str]:
        """Reasons not to (keep) driving; empty when fine. Unreadable = problem."""
        pi_min = limits.pi_start_percent if starting else limits.pi_stop_percent
        motor_min = limits.motor_start_mv if starting else limits.motor_stop_mv
        out = []
        if self.pi_percent is None:
            out.append("Pi battery unreadable")
        elif self.pi_percent < pi_min and not self.pi_plugged_in:
            out.append(f"Pi battery {self.pi_percent}% < {pi_min}%")
        if self.motor_mv is None:
            out.append("motor battery unreadable")
        elif self.motor_mv < motor_min:
            out.append(f"motor battery {self.motor_mv / 1000:.2f} V < {motor_min / 1000:.2f} V")
        return out


def load_power_limits(path: str | Path = DEFAULT_CONFIG_PATH) -> PowerLimits:
    with open(path, "rb") as f:
        p = tomllib.load(f)["power"]
    return PowerLimits(
        pi_start_percent=int(p["pi_start_percent"]),
        pi_stop_percent=int(p["pi_stop_percent"]),
        motor_start_mv=int(p["motor_start_mv"]),
        motor_stop_mv=int(p["motor_stop_mv"]),
    )


def read_pi_battery() -> tuple[int | None, bool | None]:
    """(percent, plugged_in) from the PiPower 5, or (None, None)."""
    try:
        out = subprocess.run(["pipower5", "-a"], capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None, None
    pct = re.search(r"Battery:.*?percentage:\s*(\d+)", out, re.S)
    plugged = re.search(r"plugged in:\s*(True|False)", out)
    return (int(pct.group(1)) if pct else None, plugged.group(1) == "True" if plugged else None)


def read_status(rover=None) -> PowerStatus:
    """rover: an open rover.Rover for the motor pack voltage (None = skip)."""
    pct, plugged = read_pi_battery()
    motor = None
    if rover is not None:
        try:
            motor = rover.battery_mv()
        except Exception:
            motor = None
    return PowerStatus(pct, plugged, motor)


class PowerGuard:
    """Polls every `period_s`; `.low` is set (with `.reasons`) on a stop-level reading."""

    def __init__(self, rover, limits: PowerLimits | None = None, period_s: float = 5.0):
        self.rover = rover
        self.limits = limits or load_power_limits()
        self.period_s = period_s
        self.low = threading.Event()
        self.reasons: list[str] = []
        self.last: PowerStatus | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def check_start(self) -> list[str]:
        """Problems that should stop a survey from starting (empty = OK)."""
        self.last = read_status(self.rover)
        return self.last.problems(self.limits, starting=True)

    def start(self) -> "PowerGuard":
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join()

    def _run(self) -> None:
        misses = 0
        while not self._stop.wait(self.period_s):
            self.last = read_status(self.rover)
            problems = self.last.problems(self.limits, starting=False)
            # Motor voltage sags under load: need two low readings in a row.
            misses = misses + 1 if problems else 0
            if misses >= 2:
                self.reasons = problems
                self.low.set()
