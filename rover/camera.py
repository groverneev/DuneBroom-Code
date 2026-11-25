"""Downward survey camera (Logitech C920 PRO on USB).

Surveys need every frame to look the same, so the camera's automatic focus,
exposure and white balance are locked: the camera runs on auto for a few
seconds to settle on the current scene, then those values are read back and
set as fixed manual values. Exposure is capped at `exposure_max` so frames
taken while driving aren't smeared; the lost brightness is made up with gain.

Controls go through v4l2-ctl because OpenCV's UVC control mapping is
unreliable. Units are the camera's own: exposure in 100 us, focus 0-250.
"""

from __future__ import annotations

import subprocess
import time
import tomllib
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

from rover.config import DEFAULT_CONFIG_PATH

MANUAL_EXPOSURE = 1
AUTO_EXPOSURE = 3  # "aperture priority" on the C920


class CameraError(RuntimeError):
    pass


@dataclass(frozen=True)
class CameraConfig:
    device: str
    width: int
    height: int
    fps: int
    settle_s: float
    exposure_max: int
    focus: int | None          # None = take the autofocus value at lock time
    jpeg_quality: int


@dataclass
class LockedSettings:
    exposure: int
    gain: int
    focus: int
    white_balance: int
    auto_exposure: int
    auto_gain: int


def load_camera_config(path: str | Path = DEFAULT_CONFIG_PATH) -> CameraConfig:
    with open(path, "rb") as f:
        cam = tomllib.load(f)["camera"]
    focus = int(cam.get("focus", -1))
    return CameraConfig(
        device=cam["device"],
        width=int(cam["width"]),
        height=int(cam["height"]),
        fps=int(cam["fps"]),
        settle_s=float(cam["settle_s"]),
        exposure_max=int(cam["exposure_max"]),
        focus=focus if focus >= 0 else None,
        jpeg_quality=int(cam["jpeg_quality"]),
    )


class Camera:
    def __init__(self, config: CameraConfig | None = None):
        self.config = config or load_camera_config()
        self.settings: LockedSettings | None = None
        self._cap: cv2.VideoCapture | None = None

    # ---- lifecycle ----

    def open(self, lock: bool = True) -> "Camera":
        c = self.config
        cap = cv2.VideoCapture(c.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise CameraError(f"cannot open camera {c.device}")
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, c.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, c.height)
        cap.set(cv2.CAP_PROP_FPS, c.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._cap = cap
        size = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if size != (c.width, c.height):
            self.close()
            raise CameraError(f"camera gave {size[0]}x{size[1]}, wanted {c.width}x{c.height}")
        if lock:
            self.lock()
        return self

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self) -> "Camera":
        return self.open() if self._cap is None else self

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- settings ----

    def unlock(self) -> None:
        """Back to full auto (what the camera does on its own)."""
        self._set(
            auto_exposure=AUTO_EXPOSURE,
            exposure_dynamic_framerate=0,
            focus_automatic_continuous=1,
            white_balance_automatic=1,
        )
        self.settings = None

    def lock(self) -> LockedSettings:
        """Let auto settle on the current scene, then freeze those values."""
        self.unlock()
        end = time.monotonic() + self.config.settle_s
        while time.monotonic() < end:
            self._grab()
        auto = self._get("exposure_time_absolute", "gain", "focus_absolute", "white_balance_temperature")
        exposure = max(3, min(auto["exposure_time_absolute"], self.config.exposure_max))
        gain = auto["gain"]
        if exposure < auto["exposure_time_absolute"]:
            # Shorter exposure than auto wanted: raise gain to roughly keep the
            # brightness (gain isn't linear, so this is approximate).
            ratio = auto["exposure_time_absolute"] / exposure
            gain = min(255, round(max(gain, 32) * ratio))
        focus = self.config.focus if self.config.focus is not None else auto["focus_absolute"]
        self._set(
            auto_exposure=MANUAL_EXPOSURE,
            exposure_dynamic_framerate=0,
            focus_automatic_continuous=0,
            white_balance_automatic=0,
        )
        self._set(
            exposure_time_absolute=exposure,
            gain=gain,
            focus_absolute=focus,
            white_balance_temperature=auto["white_balance_temperature"],
        )
        self.settings = LockedSettings(
            exposure=exposure,
            gain=gain,
            focus=focus,
            white_balance=auto["white_balance_temperature"],
            auto_exposure=auto["exposure_time_absolute"],
            auto_gain=auto["gain"],
        )
        # Drop frames taken before the new settings applied.
        for _ in range(5):
            self._grab()
        return self.settings

    def settings_dict(self) -> dict:
        return {"config": asdict(self.config), "locked": asdict(self.settings) if self.settings else None}

    # ---- frames ----

    def read(self) -> tuple[float, np.ndarray]:
        """(unix time just after capture, BGR frame)."""
        frame = self._grab()
        return time.time(), frame

    def encode_jpeg(self, frame: np.ndarray) -> bytes:
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality])
        if not ok:
            raise CameraError("JPEG encode failed")
        return buf.tobytes()

    # ---- internals ----

    def _grab(self) -> np.ndarray:
        if self._cap is None:
            raise CameraError("camera is not open")
        ok, frame = self._cap.read()
        if not ok:
            raise CameraError("camera read failed")
        return frame

    def _set(self, **controls: int) -> None:
        arg = ",".join(f"{k}={int(v)}" for k, v in controls.items())
        r = subprocess.run(["v4l2-ctl", "-d", self.config.device, "-c", arg], capture_output=True, text=True)
        if r.returncode != 0:
            raise CameraError(f"v4l2-ctl -c {arg}: {r.stderr.strip()}")

    def _get(self, *names: str) -> dict[str, int]:
        r = subprocess.run(
            ["v4l2-ctl", "-d", self.config.device, "-C", ",".join(names)], capture_output=True, text=True
        )
        if r.returncode != 0:
            raise CameraError(f"v4l2-ctl -C: {r.stderr.strip()}")
        values = {}
        for line in r.stdout.splitlines():
            name, _, value = line.partition(":")
            values[name.strip()] = int(value)
        return values
