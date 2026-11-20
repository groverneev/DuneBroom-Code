#!/usr/bin/env python3
"""Labeled motor-driver diagnostic: dumps driver registers, then runs a slow
sequence of single-channel spins in open-loop PWM and closed-loop speed mode,
sampling the encoders while each one runs. Tracks must be off the ground.

    python3 tools/diag_driver.py [--duty 30] [--seconds 2] [--gap 4]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rover import Rover  # noqa: E402

REGISTERS = {
    "battery (0x00, 2B)": (0x00, 2),
    "motor type (0x14)": (0x14, 1),
    "encoder polarity (0x15)": (0x15, 1),
    "pwm (0x1F, 4B)": (0x1F, 4),
    "speed (0x33, 4B)": (0x33, 4),
    "encoder totals (0x3C, 16B)": (0x3C, 16),
}


def dump(rover: Rover, label: str) -> None:
    print(f"== registers {label} ==")
    for name, (reg, n) in REGISTERS.items():
        print(f"  {name:28s} {rover.read_register(reg, n)}")


def step(rover: Rover, number: int, label: str, start, seconds: float, gap: float) -> None:
    print(f"\n[{time.strftime('%H:%M:%S')}] STEP {number}: {label}", flush=True)
    _, before = rover.raw_encoders()
    start()
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        time.sleep(0.25)
        _, raw = rover.raw_encoders()
        print(f"  t={time.monotonic() - t0:.2f}s raw={raw}", flush=True)
    rover.stop()
    time.sleep(0.4)
    _, after = rover.raw_encoders()
    print(f"  change: {[a - b for a, b in zip(after, before)]}", flush=True)
    time.sleep(gap)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duty", type=int, default=30)
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--gap", type=float, default=4.0)
    args = parser.parse_args()

    with Rover() as rover:
        print(rover.ping())
        dump(rover, "at start")
        d, s, g = args.duty, args.seconds, args.gap
        steps = [
            (f"PWM M3 (ch 2) duty {d}", lambda: rover.pwm_channel(2, d)),
            (f"PWM M4 (ch 3) duty {d}", lambda: rover.pwm_channel(3, d)),
            (f"PWM M1 (ch 0) duty {d}  [no motor plugged in]", lambda: rover.pwm_channel(0, d)),
            (f"SPEED M3 (ch 2) {d}", lambda: rover.spin_channel(2, d)),
            (f"SPEED M4 (ch 3) {d}", lambda: rover.spin_channel(3, d)),
        ]
        print(f"\nSequence (each {s}s, {g}s pause between):")
        for i, (label, _) in enumerate(steps, 1):
            print(f"  {i}. {label}")
        time.sleep(g)
        for i, (label, start) in enumerate(steps, 1):
            step(rover, i, label, start, s, g)
        dump(rover, "at end")


if __name__ == "__main__":
    main()
