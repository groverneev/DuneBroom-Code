#!/usr/bin/env python3
"""BNO055 checks and gyro-corrected turns (see rover/imu.py for wiring).

    python3 tools/imu.py watch [--seconds 20]   live heading/roll/pitch + calibration (no motors)
    python3 tools/imu.py turn --deg 90          MOVES MOTORS: turn by heading, report each step
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rover.imu import BNO055  # noqa: E402


def cmd_watch(args) -> None:
    with BNO055() as imu:
        print("BNO055 found; status", imu.status())
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            h, r, p = imu.euler()
            c = imu.calibration()
            print(f"heading {h:7.2f}  roll {r:6.2f}  pitch {p:6.2f}  "
                  f"calib sys {c['sys']} gyro {c['gyro']} accel {c['accel']}")
            time.sleep(0.5)


def cmd_turn(args) -> None:
    from rover import Rover
    from rover.heading import HeadingControl
    with BNO055() as imu, Rover() as rover:
        imu.start_tracking()
        time.sleep(1.0)  # gyro settles while still
        hc = HeadingControl(rover, imu)
        rep = hc.turn_by(args.deg, args.speed)
        for cmd, moved in rep.steps:
            print(f"  commanded {cmd:+7.2f} deg  measured {moved:+7.2f} deg")
        print(f"target {rep.target:.2f}, ended {rep.final:.2f} (error {rep.error:+.2f}); "
              f"learned ratio {hc.ratio:.3f}; imu read errors {imu.read_errors}")


def main() -> None:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    w = sub.add_parser("watch")
    w.add_argument("--seconds", type=float, default=20)
    t = sub.add_parser("turn")
    t.add_argument("--deg", type=float, required=True)
    t.add_argument("--speed", type=int, default=None)
    args = parser.parse_args()
    {"watch": cmd_watch, "turn": cmd_turn}[args.cmd](args)


if __name__ == "__main__":
    main()
