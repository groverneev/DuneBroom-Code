#!/usr/bin/env python3
"""Find a loose Uno<->driver I2C wire: polls the driver's battery register
20x per second and prints a line each second with the failure count, so
wiggling a bad wire shows up as errors. Motors never move.

    python3 tools/diag_i2c_wiggle.py --seconds 90
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rover import Rover, RoverError  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=90)
    args = parser.parse_args()

    rover = None
    for _ in range(10):
        try:
            rover = Rover().open()
            break
        except RoverError as e:
            print(f"open failed ({e}); retrying")
            time.sleep(0.5)
    if rover is None:
        sys.exit("driver never answered")

    total_ok = total_err = 0
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            ok = err = 0
            mv = None
            second_end = time.monotonic() + 1
            while time.monotonic() < second_end:
                try:
                    mv = rover.battery_mv()
                    ok += 1
                except RoverError:
                    err += 1
                time.sleep(0.05)
            total_ok += ok
            total_err += err
            flag = "  <-- ERRORS" if err else ""
            print(f"{time.strftime('%H:%M:%S')}  ok={ok:2d}  err={err:2d}  battery={mv} mV{flag}", flush=True)
    finally:
        rover.close()
    print(f"TOTAL ok={total_ok} err={total_err}")


if __name__ == "__main__":
    main()
