#!/usr/bin/env python3
"""Capture survey frames with locked camera settings (motors never move).

    python3 tools/capture.py --seconds 10            frames + odometry from the Uno
    python3 tools/capture.py --seconds 10 --no-odometry
    python3 tools/capture.py --snapshot              one locked frame, prints its path

Output goes to data/captures/<date-time>/ (see rover/capture.py).
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rover.camera import Camera  # noqa: E402
from rover.capture import Recorder, new_capture_dir  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--fps", type=float, default=5, help="frames saved per second")
    parser.add_argument("--label", default="")
    parser.add_argument("--no-odometry", action="store_true")
    parser.add_argument("--snapshot", action="store_true")
    args = parser.parse_args()

    out = new_capture_dir(args.label or ("snapshot" if args.snapshot else ""))
    with Camera() as cam:
        s = cam.settings
        print(f"locked: exposure={s.exposure} (auto wanted {s.auto_exposure}) gain={s.gain} "
              f"focus={s.focus} white_balance={s.white_balance}K")
        if args.snapshot:
            _, frame = cam.read()
            path = out / "frames" / "000001.jpg"
            path.write_bytes(cam.encode_jpeg(frame))
            print(path)
            return

        rover = None
        if not args.no_odometry:
            from rover import Rover
            rover = Rover().open()
        try:
            odometry = (lambda: rover.encoders()[1:]) if rover else None
            rec = Recorder(cam, out, args.fps, odometry).start()
            time.sleep(args.seconds)
            n = rec.stop()
        finally:
            if rover:
                rover.close()
    print(f"saved {n} frames in {out}")


if __name__ == "__main__":
    main()
