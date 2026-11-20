#!/usr/bin/env python3
"""Step-by-step motor-driver test you run at the robot. Each step waits for
Enter, runs, then asks what you saw. Your answers and the encoder readings are
saved to logs/diag_<time>.txt. Tracks must be off the ground.

    python3 tools/diag_interactive.py            # suite 1: PWM vs speed mode per port
    python3 tools/diag_interactive.py --suite 2  # hand-turn encoders, then no-encoder mode
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from rover import Rover  # noqa: E402

SECONDS = 3.0
HAND_SECONDS = 8.0
REG_MOTOR_TYPE = 0x14
MOTOR_TYPE_NO_ENCODER = 0
MOTOR_TYPE_JGB = 3

# Steps: (label, action, channel, value). Ports M1-M4 are driver channels 0-3.
#   pwm / speed: spin one channel; hand: motors off, user turns tracks by hand;
#   type: set the driver's motor type register (no motion).
SUITES = {
    1: [
        ("Raw power 30% on M3", "pwm", 2, 30),
        ("Raw power 60% on M3", "pwm", 2, 60),
        ("Raw power 60% on M4", "pwm", 3, 60),
        ("Raw power 60% on M1 (empty port)", "pwm", 0, 60),
        ("Speed mode 20 on M3", "speed", 2, 20),
        ("Speed mode 20 on M4", "speed", 3, 20),
        ("Speed mode 20 on M1 (empty port)", "speed", 0, 20),
        ("Speed mode 20 on M2 (empty port)", "speed", 1, 20),
    ],
    2: [
        ("Motors OFF: turn the LEFT track forward by hand for 8 s", "hand", 0, 0),
        ("Motors OFF: turn the RIGHT track forward by hand for 8 s", "hand", 0, 0),
        ("Set board to NO-ENCODER motor type (nothing moves)", "type", 0, MOTOR_TYPE_NO_ENCODER),
        ("Raw power 60% on M3", "pwm", 2, 60),
        ("Raw power 60% on M4", "pwm", 3, 60),
        ("Raw power 60% on M1 (empty port)", "pwm", 0, 60),
        ("Speed mode 20 on M3", "speed", 2, 20),
        ("Set board back to JGB encoder motor type (nothing moves)", "type", 0, MOTOR_TYPE_JGB),
    ],
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--suite", type=int, choices=sorted(SUITES), default=1)
    args = parser.parse_args()
    steps = SUITES[args.suite]

    logs = ROOT / "logs"
    logs.mkdir(exist_ok=True)
    log_path = logs / time.strftime(f"diag_s{args.suite}_%Y%m%d_%H%M%S.txt")
    with Rover() as rover, open(log_path, "w") as log:
        def out(line: str) -> None:
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()

        out(f"suite {args.suite}  {rover.ping()}  battery {rover.battery_mv()} mV")
        out(f"motor type register: {rover.read_register(REG_MOTOR_TYPE, 1)}")
        out("Answer each step with which track moved (left / right / both / none)")
        out("and which way (forward / backward), e.g. 'left forward' or 'none'.")
        try:
            for i, (label, action, ch, value) in enumerate(steps, 1):
                input(f"\nStep {i}/{len(steps)}: {label}. Press Enter to run... ")
                _, before = rover.raw_encoders()
                if action == "type":
                    rover.write_register(REG_MOTOR_TYPE, [value])
                    time.sleep(0.1)
                    out(f"STEP {i} {label} | register now {rover.read_register(REG_MOTOR_TYPE, 1)}")
                    continue
                if action == "hand":
                    print("  Turn it now...", flush=True)
                    t0 = time.monotonic()
                    while time.monotonic() - t0 < HAND_SECONDS:
                        time.sleep(1.0)
                        _, raw = rover.raw_encoders()
                        print(f"  t={time.monotonic() - t0:.0f}s encoders={raw}", flush=True)
                    _, after = rover.raw_encoders()
                    out(f"STEP {i} {label} | encoders before={before} after={after}")
                    continue
                if action == "pwm":
                    rover.pwm_channel(ch, value)
                else:
                    rover.spin_channel(ch, value)
                time.sleep(SECONDS)
                _, during = rover.raw_encoders()
                rover.stop()
                time.sleep(0.4)
                _, after = rover.raw_encoders()
                seen = input("  What did you see? ").strip()
                out(
                    f"STEP {i} {label} | seen: {seen} | encoders before={before} "
                    f"during={during} after={after}"
                )
        finally:
            if args.suite == 2:
                rover.write_register(REG_MOTOR_TYPE, [MOTOR_TYPE_JGB])
        out(f"battery {rover.battery_mv()} mV")
    print(f"\nSaved to {log_path}")


if __name__ == "__main__":
    main()
