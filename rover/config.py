"""Load config/rover.toml."""

from __future__ import annotations

import glob
import tomllib
from dataclasses import dataclass
from pathlib import Path

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "rover.toml"


@dataclass(frozen=True)
class RoverConfig:
    port: str
    baud: int
    left_channel: int
    right_channel: int
    left_sign: int
    right_sign: int
    closed_loop: bool
    drive_speed: int
    turn_speed: int
    ticks_per_mm: float | None
    ticks_per_degree: float | None
    turn_offset_ticks: float


def resolve_port(port: str) -> str:
    if port != "auto":
        return port
    arduinos = sorted(glob.glob("/dev/serial/by-id/*Arduino*"))
    return arduinos[0] if arduinos else "/dev/ttyACM0"


def load_config(path: str | Path = DEFAULT_CONFIG_PATH) -> RoverConfig:
    with open(path, "rb") as f:
        data = tomllib.load(f)
    serial, motors, cal = data["serial"], data["motors"], data["calibration"]
    return RoverConfig(
        port=resolve_port(serial["port"]),
        baud=int(serial["baud"]),
        left_channel=int(motors["left_channel"]),
        right_channel=int(motors["right_channel"]),
        left_sign=int(motors["left_sign"]),
        right_sign=int(motors["right_sign"]),
        closed_loop=bool(motors["closed_loop"]),
        drive_speed=int(motors["drive_speed"]),
        turn_speed=int(motors["turn_speed"]),
        ticks_per_mm=float(cal["ticks_per_mm"]) or None,
        ticks_per_degree=float(cal["ticks_per_degree"]) or None,
        turn_offset_ticks=float(cal.get("turn_offset_ticks", 0.0)),
    )
