"""Adafruit BNO055 IMU over UART: heading for turns and straight lanes.

Wiring (PS1 high selects UART mode; the Pi's I2C clock stretching is
unreliable with this chip):
    BNO055 VIN -> Pi 3.3 V (pin 1)    GND -> Pi GND (pin 6)
    BNO055 SDA (UART TX) -> Pi GPIO15 RXD (pin 10)
    BNO055 SCL (UART RX) -> Pi GPIO14 TXD (pin 8)
    BNO055 PS1 -> 3.3 V (Vin)
Pi side: `dtparam=uart0=on` in /boot/firmware/config.txt gives /dev/ttyAMA0.

Mode IMUPLUS (gyro + accelerometer fusion, no magnetometer): heading is
relative to where the rover pointed at start, and the motors and steel
chassis can't bend it. Drift is a fraction of a degree per minute.

`yaw` is unwrapped and clockwise-positive (seen from above), matching
Rover.turn_deg: it goes 359 -> 360 -> 361, not back to 0.

UART protocol (datasheet 4.7): write  AA 00 reg len data -> EE 01
                                 read   AA 01 reg len      -> BB len data | EE err
"""

from __future__ import annotations

import threading
import time

DEFAULT_PORT = "/dev/ttyAMA0"
BAUD = 115200

CHIP_ID = 0xA0
REG_CHIP_ID = 0x00
REG_PAGE_ID = 0x07
REG_EUL_HEADING = 0x1A   # 3 x int16 LE: heading, roll, pitch; 1/16 deg
REG_GYR_Z = 0x18
REG_CALIB_STAT = 0x35
REG_SYS_STATUS = 0x39
REG_SYS_ERR = 0x3A
REG_UNIT_SEL = 0x3B
REG_OPR_MODE = 0x3D
REG_PWR_MODE = 0x3E
REG_SYS_TRIGGER = 0x3F
REG_AXIS_MAP_CONFIG = 0x41
REG_AXIS_MAP_SIGN = 0x42

MODE_CONFIG = 0x00
MODE_IMUPLUS = 0x08
MODE_NDOF = 0x0C

WRITE_OK = 0x01
RETRYABLE = {0x07}       # BUS_OVER_RUN: chip busy, a retry usually works


class ImuError(RuntimeError):
    pass


def unwrap(prev_raw: float, prev_yaw: float, raw: float) -> float:
    """Continue an unwrapped yaw from a new 0-360 reading."""
    d = (raw - prev_raw + 180.0) % 360.0 - 180.0
    return prev_yaw + d


class BNO055:
    def __init__(self, port: str = DEFAULT_PORT, mode: int = MODE_IMUPLUS, serial_factory=None):
        self.port = port
        self.mode = mode
        self._serial_factory = serial_factory
        self._ser = None
        self._lock = threading.Lock()
        # Background heading tracking
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._yaw: float | None = None
        self._raw: float | None = None
        self._yaw_time = 0.0
        self.read_errors = 0

    # ---- lifecycle ----

    def open(self) -> "BNO055":
        if self._serial_factory:
            self._ser = self._serial_factory()
        else:
            import serial
            self._ser = serial.Serial(self.port, BAUD, timeout=0.1)
        self._ser.reset_input_buffer()
        try:
            chip = self._read_retry(REG_CHIP_ID, 1, tries=10)[0]
        except ImuError as e:
            self.close()
            raise ImuError(f"no BNO055 on {self.port} ({e}): check TX/RX crossed, PS1 to 3.3 V, power") from None
        if chip != CHIP_ID:
            raise ImuError(f"chip id 0x{chip:02x}, expected 0x{CHIP_ID:02x}")
        self._write(REG_OPR_MODE, [MODE_CONFIG])
        time.sleep(0.03)
        self._write(REG_PAGE_ID, [0])
        self._write(REG_PWR_MODE, [0])          # normal power
        self._write(REG_UNIT_SEL, [0x00])       # degrees, m/s^2, Celsius
        self._write(REG_SYS_TRIGGER, [0x00])    # internal oscillator
        self._write(REG_OPR_MODE, [self.mode])
        time.sleep(0.03)
        return self

    def close(self) -> None:
        self.stop_tracking()
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def __enter__(self) -> "BNO055":
        return self.open()

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- readings ----

    def heading(self) -> float:
        """Fused heading 0-360 deg (clockwise from the start direction in IMUPLUS)."""
        return self._int16(self._read_retry(REG_EUL_HEADING, 2)) / 16.0

    def euler(self) -> tuple[float, float, float]:
        """(heading, roll, pitch) in degrees."""
        d = self._read_retry(REG_EUL_HEADING, 6)
        return tuple(self._int16(d[i:i + 2]) / 16.0 for i in (0, 2, 4))

    def calibration(self) -> dict[str, int]:
        """0 (uncalibrated) .. 3 (fully) per part. In IMUPLUS, gyro matters most:
        it reaches 3 after a few seconds standing still."""
        v = self._read_retry(REG_CALIB_STAT, 1)[0]
        return {"sys": v >> 6 & 3, "gyro": v >> 4 & 3, "accel": v >> 2 & 3, "mag": v & 3}

    def status(self) -> tuple[int, int]:
        """(SYS_STATUS, SYS_ERR); 5 = fusion running, 1 = error (see SYS_ERR)."""
        d = self._read_retry(REG_SYS_STATUS, 2)
        return d[0], d[1]

    # ---- continuous yaw ----

    def start_tracking(self, period_s: float = 0.02) -> "BNO055":
        """Poll the heading in the background so `yaw` is always fresh."""
        self._raw = self.heading()
        self._yaw = self._raw
        self._yaw_time = time.monotonic()
        self._stop.clear()
        self._thread = threading.Thread(target=self._track, args=(period_s,), daemon=True)
        self._thread.start()
        return self

    def stop_tracking(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join()
            self._thread = None

    @property
    def yaw(self) -> float:
        """Unwrapped heading (deg, clockwise +). Raises if the reading is stale."""
        if self._yaw is None:
            raise ImuError("tracking not started")
        age = time.monotonic() - self._yaw_time
        if age > 0.5:
            raise ImuError(f"heading is {age:.1f} s old")
        return self._yaw

    def _track(self, period_s: float) -> None:
        while not self._stop.wait(period_s):
            try:
                raw = self.heading()
            except ImuError:
                self.read_errors += 1
                continue
            self._yaw = unwrap(self._raw, self._yaw, raw)
            self._raw = raw
            self._yaw_time = time.monotonic()

    # ---- protocol ----

    @staticmethod
    def _int16(b: bytes | list[int]) -> int:
        v = b[0] | b[1] << 8
        return v - 65536 if v & 0x8000 else v

    def _read_retry(self, reg: int, length: int, tries: int = 3) -> bytes:
        err: ImuError | None = None
        for _ in range(tries):
            try:
                return self._read(reg, length)
            except ImuError as e:
                err = e
                time.sleep(0.01)
        raise err

    def _read(self, reg: int, length: int) -> bytes:
        with self._lock:
            self._ser.reset_input_buffer()
            self._ser.write(bytes([0xAA, 0x01, reg, length]))
            head = self._ser.read(2)
            if len(head) < 2:
                raise ImuError(f"read 0x{reg:02x}: no reply")
            if head[0] == 0xEE:
                raise ImuError(f"read 0x{reg:02x}: error 0x{head[1]:02x}")
            if head[0] != 0xBB or head[1] != length:
                raise ImuError(f"read 0x{reg:02x}: bad header {head.hex()}")
            data = self._ser.read(length)
            if len(data) < length:
                raise ImuError(f"read 0x{reg:02x}: short data")
            return data

    def _write(self, reg: int, data: list[int], tries: int = 5) -> None:
        for _ in range(tries):
            with self._lock:
                self._ser.reset_input_buffer()
                self._ser.write(bytes([0xAA, 0x00, reg, len(data), *data]))
                resp = self._ser.read(2)
            if len(resp) == 2 and resp[0] == 0xEE and resp[1] == WRITE_OK:
                return
            if len(resp) == 2 and resp[0] == 0xEE and resp[1] not in RETRYABLE:
                raise ImuError(f"write 0x{reg:02x}: error 0x{resp[1]:02x}")
            time.sleep(0.01)
        raise ImuError(f"write 0x{reg:02x}: no OK after {tries} tries")
