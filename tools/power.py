#!/usr/bin/env python3
"""Show both battery levels and whether a survey could start (motors never move).

    python3 tools/power.py
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rover import Rover, RoverError  # noqa: E402
from rover.power import load_power_limits, read_status  # noqa: E402


def main() -> None:
    limits = load_power_limits()
    try:
        with Rover() as rover:
            status = read_status(rover)
    except RoverError as e:
        print(f"Uno/driver not answering ({e}); motor battery unknown")
        status = read_status(None)
    pi = "unknown" if status.pi_percent is None else f"{status.pi_percent}%"
    plug = " (charging input plugged in)" if status.pi_plugged_in else ""
    motor = "unknown" if status.motor_mv is None else f"{status.motor_mv / 1000:.2f} V"
    print(f"Pi battery:    {pi}{plug}")
    print(f"Motor battery: {motor}")
    problems = status.problems(limits)
    print("Survey start: OK" if not problems else "Survey start: NO - " + "; ".join(problems))


if __name__ == "__main__":
    main()
