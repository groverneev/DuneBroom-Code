#!/usr/bin/env python3
"""Command-line tests for the rover's motors and encoders.

    python3 tools/rover_cli.py ping            firmware version and battery
    python3 tools/rover_cli.py enc             encoder readings
    python3 tools/rover_cli.py channels        spin each driver channel to find the tracks
    python3 tools/rover_cli.py drive --ticks 3000   (or --mm 500 once calibrated)
    python3 tools/rover_cli.py turn --ticks 1500    (or --deg 90; + = clockwise)
    python3 tools/rover_cli.py run --left 70 --right 70 --seconds 2   timed, no encoders needed
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rover import Rover, load_config  # noqa: E402


def cmd_ping(rover: Rover, args) -> None:
    print(rover.ping())
    print(f"battery: {rover.battery_mv()} mV")
    print(f"port: {rover.config.port}")


def cmd_enc(rover: Rover, args) -> None:
    ms, left, right = rover.encoders()
    _, raw = rover.raw_encoders()
    print(f"t={ms} ms  left={left}  right={right}  raw={raw}")


def cmd_channels(rover: Rover, args) -> None:
    print(f"Spinning each driver channel at speed {args.speed} for {args.seconds} s.")
    for ch in range(4):
        _, before = rover.raw_encoders()
        rover.spin_channel(ch, args.speed)
        time.sleep(args.seconds)
        rover.stop()
        time.sleep(0.3)
        _, after = rover.raw_encoders()
        deltas = [a - b for a, b in zip(after, before)]
        print(f"channel {ch} (M{ch + 1}): encoder change per channel {deltas}")
    print("Note which track moved for each channel and which way it turned.")


def _report(result) -> None:
    print(
        f"{result.reason}: left={result.left_ticks} right={result.right_ticks} "
        f"mean={result.mean_ticks:.0f} ticks"
    )


def cmd_drive(rover: Rover, args) -> None:
    if args.mm is not None:
        _report(rover.drive_mm(args.mm, args.speed))
    else:
        _report(rover.drive_ticks(args.ticks, args.speed))


def cmd_turn(rover: Rover, args) -> None:
    if args.deg is not None:
        _report(rover.turn_deg(args.deg, args.speed))
    else:
        _report(rover.rotate_ticks(args.ticks, args.speed))


def cmd_run(rover: Rover, args) -> None:
    rover.run_for(args.left, args.right, args.seconds)
    print(f"ran left={args.left} right={args.right} for {args.seconds} s")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default=None, help="path to rover.toml")
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("ping").set_defaults(func=cmd_ping)
    sub.add_parser("enc").set_defaults(func=cmd_enc)

    p = sub.add_parser("channels")
    p.add_argument("--speed", type=int, default=30)
    p.add_argument("--seconds", type=float, default=1.0)
    p.set_defaults(func=cmd_channels)

    p = sub.add_parser("drive")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--ticks", type=int)
    g.add_argument("--mm", type=float)
    p.add_argument("--speed", type=int, default=None)
    p.set_defaults(func=cmd_drive)

    p = sub.add_parser("turn")
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--ticks", type=int)
    g.add_argument("--deg", type=float)
    p.add_argument("--speed", type=int, default=None)
    p.set_defaults(func=cmd_turn)

    p = sub.add_parser("run")
    p.add_argument("--left", type=int, required=True)
    p.add_argument("--right", type=int, required=True)
    p.add_argument("--seconds", type=float, required=True)
    p.set_defaults(func=cmd_run)

    args = parser.parse_args()
    config = load_config(args.config) if args.config else load_config()
    with Rover(config) as rover:
        args.func(rover, args)


if __name__ == "__main__":
    main()
