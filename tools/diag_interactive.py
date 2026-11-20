#!/usr/bin/env python3
"""Step-by-step motor-driver test you run at the robot. Each step waits for
Enter, spins for a few seconds, then asks what you saw. Your answers and the
encoder readings are saved to logs/diag_<time>.txt. Tracks must be off the ground.

    python3 tools/diag_interactive.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from rover import Rover  # noqa: E402

SECONDS = 3.0

# (label, mode, channel, value). Ports M1-M4 are driver channels 0-3.
STEPS = [
    ("Raw power 30% on M3", "pwm", 2, 30),
    ("Raw power 60% on M3", "pwm", 2, 60),
    ("Raw power 60% on M4", "pwm", 3, 60),
    ("Raw power 60% on M1 (empty port)", "pwm", 0, 60),
    ("Speed mode 20 on M3", "speed", 2, 20),
    ("Speed mode 20 on M4", "speed", 3, 20),
    ("Speed mode 20 on M1 (empty port)", "speed", 0, 20),
    ("Speed mode 20 on M2 (empty port)", "speed", 1, 20),
]


def main() -> None:
    logs = ROOT / "logs"
    logs.mkdir(exist_ok=True)
    log_path = logs / time.strftime("diag_%Y%m%d_%H%M%S.txt")
    with Rover() as rover, open(log_path, "w") as log:
        def out(line: str) -> None:
            print(line, flush=True)
            log.write(line + "\n")
            log.flush()

        out(f"{rover.ping()}  battery {rover.battery_mv()} mV")
        out("Answer each step with which track moved (left / right / both / none)")
        out("and which way (forward / backward), e.g. 'left forward' or 'none'.")
        for i, (label, mode, ch, value) in enumerate(STEPS, 1):
            input(f"\nStep {i}/{len(STEPS)}: {label}. Press Enter to run... ")
            _, before = rover.raw_encoders()
            if mode == "pwm":
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
        out(f"battery {rover.battery_mv()} mV")
    print(f"\nSaved to {log_path}")


if __name__ == "__main__":
    main()
