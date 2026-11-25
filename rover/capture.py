"""Save survey frames with timestamps and odometry.

A capture folder holds:
    frames/000001.jpg ...   the photos
    frames.csv              index, file, unix time, left/right ticks (blank without odometry)
    camera.json             camera config and the locked settings
"""

from __future__ import annotations

import csv
import json
import threading
import time
from collections.abc import Callable
from pathlib import Path

from rover.camera import Camera

DATA_DIR = Path(__file__).resolve().parent.parent / "data" / "captures"

# Returns (left_ticks, right_ticks), or None when there is no odometry.
Odometry = Callable[[], "tuple[int, int] | None"]


def new_capture_dir(label: str = "") -> Path:
    name = time.strftime("%Y%m%d-%H%M%S") + (f"-{label}" if label else "")
    path = DATA_DIR / name
    (path / "frames").mkdir(parents=True)
    return path


class Recorder:
    """Saves frames at `save_fps` on a background thread until stop()."""

    def __init__(self, camera: Camera, out_dir: Path, save_fps: float, odometry: Odometry | None = None):
        self.camera = camera
        self.out_dir = out_dir
        self.period = 1.0 / save_fps
        self.odometry = odometry
        self.count = 0
        self.error: Exception | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> "Recorder":
        (self.out_dir / "camera.json").write_text(json.dumps(self.camera.settings_dict(), indent=2))
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> int:
        self._stop.set()
        if self._thread:
            self._thread.join()
        if self.error:
            raise self.error
        return self.count

    def _run(self) -> None:
        try:
            with open(self.out_dir / "frames.csv", "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["index", "file", "unix_time", "left_ticks", "right_ticks"])
                next_t = time.monotonic()
                while not self._stop.is_set():
                    t, frame = self.camera.read()  # always read: keeps the buffer fresh
                    if time.monotonic() < next_t:
                        continue
                    next_t += self.period
                    odo = self.odometry() if self.odometry else None
                    self.count += 1
                    name = f"{self.count:06d}.jpg"
                    (self.out_dir / "frames" / name).write_bytes(self.camera.encode_jpeg(frame))
                    writer.writerow([self.count, f"frames/{name}", f"{t:.3f}", *(odo or ("", ""))])
                    f.flush()
        except Exception as e:  # surfaced by stop()
            self.error = e
