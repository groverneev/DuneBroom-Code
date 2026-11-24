"""Serial driver for the motor controller firmware in firmware/dunebroom_motor.

The firmware stops the motors if it hears nothing for 600 ms, so a background
thread sends a heartbeat while the Rover is open. If this process dies, the
rover stops.
"""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import dataclass

import serial

from rover.config import RoverConfig, load_config

HEARTBEAT_S = 0.2
REPLY_TIMEOUT_S = 1.0
BOOT_TIMEOUT_S = 5.0


class RoverError(RuntimeError):
    pass


@dataclass(frozen=True)
class MoveResult:
    kind: str  # "D" drive, "R" rotate, "M" manual, "C" channel test
    reason: str  # TARGET, STOP, WATCHDOG, STALL or I2C
    left_ticks: int
    right_ticks: int

    @property
    def reached_target(self) -> bool:
        return self.reason == "TARGET"

    @property
    def mean_ticks(self) -> float:
        return (abs(self.left_ticks) + abs(self.right_ticks)) / 2


class Rover:
    def __init__(self, config: RoverConfig | None = None):
        self.config = config or load_config()
        self._ser: serial.Serial | None = None
        self._replies: queue.Queue[str] = queue.Queue()
        self._done: queue.Queue[MoveResult] = queue.Queue()
        self._write_lock = threading.Lock()
        self._request_lock = threading.Lock()
        self._closing = threading.Event()
        self._threads: list[threading.Thread] = []

    # ---- lifecycle ----

    def open(self) -> "Rover":
        # Opening the port resets the Uno; wait for its READY banner.
        self._ser = serial.Serial(self.config.port, self.config.baud, timeout=0.1)
        deadline = time.monotonic() + BOOT_TIMEOUT_S
        while not self._readline().startswith("READY"):
            if time.monotonic() > deadline:
                self._ser.close()
                raise RoverError(f"no READY from motor controller on {self.config.port}")
        self._closing.clear()
        for target in (self._read_loop, self._heartbeat_loop):
            t = threading.Thread(target=target, daemon=True)
            t.start()
            self._threads.append(t)
        c = self.config
        self._request(f"MAP {c.left_channel} {c.right_channel} {c.left_sign} {c.right_sign}", "OK MAP")
        self._request(f"CL {int(c.closed_loop)}", "OK CL")
        return self

    def close(self) -> None:
        if self._ser is None:
            return
        try:
            self.stop()
        except (RoverError, serial.SerialException):
            pass
        self._closing.set()
        for t in self._threads:
            t.join(timeout=1)
        self._threads.clear()
        self._ser.close()
        self._ser = None

    def __enter__(self) -> "Rover":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- queries ----

    def ping(self) -> str:
        return self._request("PING", "PONG")

    def encoders(self) -> tuple[int, int, int]:
        """(arduino_millis, left_ticks, right_ticks), forward-positive."""
        _, ms, left, right = self._request("E", "ENC").split()
        return int(ms), int(left), int(right)

    def raw_encoders(self) -> tuple[int, list[int]]:
        """(arduino_millis, [ch0, ch1, ch2, ch3]) straight from the driver."""
        parts = self._request("X", "RAW").split()
        return int(parts[1]), [int(v) for v in parts[2:6]]

    def battery_mv(self) -> int:
        return int(self._request("B", "BAT").split()[1])

    # ---- motion ----

    def drive_ticks(self, ticks: int, speed: int | None = None, timeout: float = 120) -> MoveResult:
        return self._move(f"D {int(ticks)} {speed or self.config.drive_speed}", "OK D", timeout)

    def rotate_ticks(self, ticks: int, speed: int | None = None, timeout: float = 60) -> MoveResult:
        """Positive ticks rotate clockwise (to the right) seen from above."""
        return self._move(f"R {int(ticks)} {speed or self.config.turn_speed}", "OK R", timeout)

    def drive_mm(self, mm: float, speed: int | None = None, timeout: float = 120) -> MoveResult:
        if not self.config.ticks_per_mm:
            raise RoverError("ticks_per_mm is not calibrated in config/rover.toml")
        return self.drive_ticks(round(mm * self.config.ticks_per_mm), speed, timeout)

    def turn_deg(self, degrees: float, speed: int | None = None, timeout: float = 60) -> MoveResult:
        """Positive degrees turn clockwise (to the right) seen from above."""
        if not self.config.ticks_per_degree:
            raise RoverError("ticks_per_degree is not calibrated in config/rover.toml")
        if degrees == 0:
            raise RoverError("turn of 0 degrees")
        # Each turn loses a fixed number of ticks before the chassis rotates.
        ticks = abs(degrees) * self.config.ticks_per_degree + self.config.turn_offset_ticks
        return self.rotate_ticks(round(ticks if degrees > 0 else -ticks), speed, timeout)

    def set_speeds(self, left: int, right: int) -> None:
        """Run the tracks continuously until changed or stop() (teleop)."""
        self._request(f"M {int(left)} {int(right)}", "OK M")

    def run_for(self, left: int, right: int, seconds: float) -> None:
        """Run the tracks at fixed speeds for a time, then stop (no encoders needed)."""
        self.set_speeds(left, right)
        try:
            time.sleep(seconds)
        finally:
            self.stop()

    def spin_channel(self, channel: int, speed: int) -> None:
        """Spin one raw driver channel, ignoring the track mapping (wiring test)."""
        self._request(f"C {int(channel)} {int(speed)}", "OK C")

    def pwm_channel(self, channel: int, duty: int) -> None:
        """Open-loop PWM on one raw driver channel (no encoder feedback; diagnostics)."""
        self._request(f"P {int(channel)} {int(duty)}", "OK P")

    def read_register(self, reg: int, length: int) -> list[int]:
        """Raw bytes from a motor driver register (diagnostics)."""
        return [int(v) for v in self._request(f"RD {int(reg)} {int(length)}", "REG").split()[2:]]

    def write_register(self, reg: int, values: list[int]) -> None:
        """Write 1-4 raw bytes to a motor driver register (diagnostics)."""
        self._request("WR " + " ".join(str(int(v)) for v in [reg, *values]), "OK WR")

    def stop(self) -> None:
        self._request("S", "OK S")

    # ---- internals ----

    def _move(self, command: str, ack: str, timeout: float) -> MoveResult:
        _drain(self._done)
        self._request(command, ack)
        try:
            return self._done.get(timeout=timeout)
        except queue.Empty:
            self.stop()
            raise RoverError(f"'{command}' did not finish within {timeout} s; stopped") from None

    def _request(self, command: str, expect: str) -> str:
        with self._request_lock:
            _drain(self._replies)
            self._send(command)
            deadline = time.monotonic() + REPLY_TIMEOUT_S
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise RoverError(f"no reply to '{command}'")
                try:
                    line = self._replies.get(timeout=remaining)
                except queue.Empty:
                    continue
                if line.startswith(expect):
                    return line
                if line.startswith("ERR"):
                    raise RoverError(f"'{command}' failed: {line}")

    def _send(self, line: str) -> None:
        if self._ser is None:
            raise RoverError("rover is not open")
        with self._write_lock:
            self._ser.write((line + "\n").encode("ascii"))

    def _readline(self) -> str:
        return self._ser.readline().decode("ascii", errors="replace").strip()

    def _read_loop(self) -> None:
        while not self._closing.is_set():
            try:
                line = self._readline()
            except serial.SerialException:
                return
            if not line:
                continue
            if line.startswith("DONE "):
                _, kind, reason, left, right = line.split()
                self._done.put(MoveResult(kind, reason, int(left), int(right)))
            elif line.startswith("READY"):
                # The Arduino rebooted (brownout or reset): its channel map is lost.
                self._replies.put("ERR RESET")
            else:
                self._replies.put(line)

    def _heartbeat_loop(self) -> None:
        while not self._closing.wait(HEARTBEAT_S):
            try:
                self._send("H")
            except (RoverError, serial.SerialException):
                return


def _drain(q: queue.Queue) -> None:
    while True:
        try:
            q.get_nowait()
        except queue.Empty:
            return
