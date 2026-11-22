#!/usr/bin/env python3
"""Print the encoder_test sketch's output for a while (run on the Pi).

    python3 tools/encoder_watch.py --seconds 30
Shows a line whenever count, edges or pin levels change.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from rover.config import resolve_port  # noqa: E402

p = argparse.ArgumentParser()
p.add_argument("--seconds", type=float, default=30)
p.add_argument("--port", default="auto")
args = p.parse_args()

with serial.Serial(resolve_port(args.port), 115200, timeout=0.5) as ser:
    end = time.monotonic() + args.seconds
    last = None
    while time.monotonic() < end:
        line = ser.readline().decode("ascii", "replace").strip()
        if not line.startswith("ENC"):
            if line:
                print(line)
            continue
        _, ms, count, ea, eb, a, b = line.split()
        state = (count, ea, eb, a, b)
        if state != last:
            print(f"t={int(ms)/1000:7.2f}s count={count} edgesA={ea} edgesB={eb} A={a} B={b}", flush=True)
            last = state
    print(f"final: count={last[0]} edgesA={last[1]} edgesB={last[2]}" if last else "no ENC lines received")
