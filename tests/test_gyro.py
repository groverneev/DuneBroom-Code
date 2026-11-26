#!/usr/bin/env python3
"""IMU protocol and heading control against simulated hardware.

    python3 tests/test_gyro.py
"""

from __future__ import annotations

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import rover.heading as heading_mod  # noqa: E402
from rover.heading import HeadingControl, HeadingError  # noqa: E402
from rover.imu import BNO055, ImuError, unwrap  # noqa: E402

heading_mod.SETTLE_S = 0


class FakeBNO055Serial:
    """Speaks the BNO055 UART protocol from a register array."""

    def __init__(self, busy_writes: int = 0):
        self.regs = bytearray(128)
        self.regs[0x00] = 0xA0
        self.regs[0x35] = 0b00110000   # gyro calibrated
        self.out = bytearray()
        self.busy_writes = busy_writes

    def set_heading(self, deg: float) -> None:
        v = round(deg * 16) & 0xFFFF
        self.regs[0x1A], self.regs[0x1B] = v & 0xFF, v >> 8

    def reset_input_buffer(self):
        self.out.clear()

    def write(self, b: bytes):
        assert b[0] == 0xAA
        op, reg, n = b[1], b[2], b[3]
        if op == 0x00:
            if self.busy_writes:
                self.busy_writes -= 1
                self.out += bytes([0xEE, 0x07])
                return
            self.regs[reg:reg + n] = b[4:4 + n]
            self.out += bytes([0xEE, 0x01])
        else:
            self.out += bytes([0xBB, n]) + self.regs[reg:reg + n]

    def read(self, n: int) -> bytes:
        d, self.out = bytes(self.out[:n]), self.out[n:]
        return d

    def close(self):
        pass


def test_protocol():
    fake = FakeBNO055Serial(busy_writes=2)
    imu = BNO055(serial_factory=lambda: fake).open()
    assert fake.regs[0x3D] == 0x08, "IMUPLUS mode set"
    fake.set_heading(123.4375)
    assert imu.heading() == 123.4375
    assert imu.calibration()["gyro"] == 3
    fake.regs[0x00] = 0x55
    try:
        BNO055(serial_factory=lambda: fake).open()
        raise AssertionError("bad chip id accepted")
    except ImuError:
        pass


def test_unwrap():
    assert unwrap(350, 350, 10) == 370
    assert unwrap(10, 370, 350) == 350
    assert unwrap(5, -355, 355) == -365


class SimRobot:
    """Skid turns land within +-15% of the command, plus a random offset."""

    def __init__(self, scale: float, seed: int):
        self.yaw = 0.0
        self.scale = scale
        self.rng = random.Random(seed)
        self.turns = 0

    def turn_deg(self, deg, speed=None):
        self.turns += 1
        if abs(deg) >= 2.5:   # tiny commands don't break static friction
            self.yaw += deg * self.scale * self.rng.uniform(0.9, 1.1) + self.rng.uniform(-1, 1)
        return type("R", (), {"reason": "TARGET"})()

    def drive_mm(self, mm, speed=None):
        self.yaw += self.rng.uniform(-3, 3)   # lane drift
        return type("R", (), {"reason": "TARGET", "reached_target": True})()


def test_turns():
    for scale in (0.8, 1.0, 1.25):
        for seed in range(20):
            sim = SimRobot(scale, seed)
            hc = HeadingControl(sim, sim)
            for target in (90, 180, 90, 0, -90):
                rep = hc.turn_to(target)
                assert abs(rep.error) <= 5.0, (scale, seed, rep)
    # learns the ratio
    assert 1.1 < hc.ratio < 1.4, hc.ratio


def test_lane():
    sim = SimRobot(1.0, 3)
    hc = HeadingControl(sim, sim)
    errs = hc.drive_straight(2000, heading=0)
    assert len(errs) == 4
    # every segment starts square, so drift never accumulates past one segment
    assert all(abs(e) <= 1.5 + 3 for e in errs)


def test_stuck():
    class Stuck(SimRobot):
        def turn_deg(self, deg, speed=None):
            return type("R", (), {"reason": "STALL"})()
    sim = Stuck(1.0, 0)
    try:
        HeadingControl(sim, sim).turn_to(90)
        raise AssertionError("stuck turn not reported")
    except HeadingError:
        pass


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
