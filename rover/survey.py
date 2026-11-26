"""Survey a rectangular plot in serpentine lanes, saving frames as it goes.

Plot frame (mm): the rover starts at the near corner facing +y (along the
plot); x grows toward the side the lanes advance to. Headings are the IMU's
unwrapped yaw relative to the start, clockwise +, so lane 0 is 0 deg, lane 1
is 180 (or -180 when lanes go left), and the sideways shift between lanes is
+90 (or -90).

Geometry is planned for the camera's footprint, not the rover's body: lane
centres are spaced by the footprint width less the overlap, and each lane is
the plot length less the footprint length (the view covers half a footprint
past each end of the robot's track). A camera mounted ahead of the turning
centre loses 2 x camera_ahead_mm of the plot at each lane end, so the rover
drives that much further before turning.

The survey only drives and records (frames, odometry, heading, phase);
analysis runs afterwards on the saved folder.
"""

from __future__ import annotations

import json
import math
import threading
import time
import tomllib
from dataclasses import asdict, dataclass, field
from pathlib import Path

from rover.config import DEFAULT_CONFIG_PATH

SURVEY_DIR = Path(__file__).resolve().parent.parent / "data" / "surveys"
MIN_MOVE_MM = 10            # skip moves shorter than this
LOOKAHEAD_SEGMENTS = 2      # steer back onto a lane's line over this many segments
MAX_STEER_DEG = 10


@dataclass(frozen=True)
class SurveySettings:
    side: str
    side_overlap: float
    footprint_across_mm: float
    footprint_along_mm: float
    camera_ahead_mm: float
    speed: int
    segment_mm: float
    save_fps: float


def load_survey_settings(path: str | Path = DEFAULT_CONFIG_PATH,
                         calibration_path: Path | None = None) -> SurveySettings:
    with open(path, "rb") as f:
        s = tomllib.load(f)["survey"]
    across, along = float(s["footprint_across_mm"]), float(s["footprint_along_mm"])
    from rover.camera_model import CALIBRATION_PATH, CameraModel
    cal = calibration_path or CALIBRATION_PATH
    if cal.exists():
        g = CameraModel.load(cal).ground
        if g:
            w, h = g.footprint_w_mm, g.footprint_h_mm
            across, along = (w, h) if s["camera_across"] == "width" else (h, w)
    if s["side"] not in ("right", "left"):
        raise ValueError("survey side must be 'right' or 'left'")
    return SurveySettings(
        side=s["side"], side_overlap=float(s["side_overlap"]),
        footprint_across_mm=across, footprint_along_mm=along,
        camera_ahead_mm=float(s["camera_ahead_mm"]), speed=int(s["speed"]),
        segment_mm=float(s["segment_mm"]), save_fps=float(s["save_fps"]),
    )


# ---- plan ----

@dataclass
class Lane:
    index: int
    heading: float        # yaw target while driving it
    x_mm: float           # footprint centre across the plot
    length_mm: float      # rover travel along it


@dataclass
class Plan:
    width_mm: float       # across the lanes
    length_mm: float      # along the lanes
    settings: SurveySettings
    lanes: list[Lane]
    shift_heading: float  # yaw for the sideways move between lanes

    def track(self, lane: Lane) -> tuple[float, float, float]:
        """(x, y_start, y_end) of the rover's turning centre for a lane, in
        odometry coordinates (start = origin, +y along lane 0, +x to the right)."""
        sign = 1 if self.settings.side == "right" else -1
        x = sign * (lane.x_mm - self.lanes[0].x_mm)
        extra = self.end_extra_mm
        if lane.index % 2 == 0:
            return x, 0.0, lane.length_mm
        return x, lane.length_mm + extra, extra

    @property
    def lane_gaps(self) -> list[float]:
        return [b.x_mm - a.x_mm for a, b in zip(self.lanes, self.lanes[1:])]

    @property
    def end_extra_mm(self) -> float:
        return 2 * self.settings.camera_ahead_mm

    def drive_mm(self) -> float:
        lanes = sum(l.length_mm for l in self.lanes)
        return lanes + sum(self.lane_gaps) + abs(self.end_extra_mm) * (len(self.lanes) - 1)

    def estimate_s(self, mm_per_s: float = 280, deg_per_s: float = 120, per_move_s: float = 1.0) -> float:
        """Rough time: driving + turning + a pause per move (settle, corrections)."""
        segs = sum(max(1, math.ceil(l.length_mm / self.settings.segment_mm)) for l in self.lanes)
        turns = 2 * (len(self.lanes) - 1)
        moves = segs + turns * 2 + 2 * (len(self.lanes) - 1)
        return self.drive_mm() / mm_per_s + turns * 90 / deg_per_s + moves * per_move_s

    def covered_area_m2(self) -> float:
        """Area the planned footprints sweep inside the plot."""
        s = self.settings
        if not self.lanes:
            return 0.0
        across = min(self.width_mm, self.lanes[-1].x_mm - self.lanes[0].x_mm + s.footprint_across_mm)
        along = min(self.length_mm, self.lanes[0].length_mm + s.footprint_along_mm)
        return across * along / 1e6

    def describe(self) -> str:
        s = self.settings
        gaps = self.lane_gaps
        lines = [
            f"plot {self.width_mm / 1000:.2f} m wide x {self.length_mm / 1000:.2f} m long "
            f"({self.width_mm * self.length_mm / 1e6:.2f} m^2), lanes go {s.side}",
            f"camera view {s.footprint_across_mm:.0f} x {s.footprint_along_mm:.0f} mm "
            f"(across x along), {s.side_overlap:.0%} side overlap",
            f"{len(self.lanes)} lanes of {self.lanes[0].length_mm / 1000:.2f} m"
            + (f", {min(gaps):.0f}-{max(gaps):.0f} mm apart" if gaps else ""),
            f"covers {self.covered_area_m2():.2f} m^2; drives {self.drive_mm() / 1000:.1f} m; "
            f"about {self.estimate_s() / 60:.1f} min",
        ]
        return "\n".join(lines)


def make_plan(width_mm: float, length_mm: float, settings: SurveySettings) -> Plan:
    s = settings
    if width_mm <= 0 or length_mm <= 0:
        raise ValueError("plot size must be positive")
    across, along = s.footprint_across_mm, s.footprint_along_mm
    spacing = across * (1 - s.side_overlap)
    if width_mm <= across:
        centres = [width_mm / 2]
    else:
        n = math.ceil((width_mm - across) / spacing) + 1
        centres = [min(across / 2 + i * spacing, width_mm - across / 2) for i in range(n)]
    sign = 1 if s.side == "right" else -1
    lane_len = max(0.0, length_mm - along)
    lanes = [Lane(i, 0.0 if i % 2 == 0 else 180.0 * sign, c, lane_len) for i, c in enumerate(centres)]
    return Plan(width_mm, length_mm, s, lanes, 90.0 * sign)


# ---- position ----

class Odometer:
    """Dead reckoning: track travel (mean of both encoders) along the IMU yaw."""

    def __init__(self, rover, imu, ticks_per_mm: float):
        self.rover = rover
        self.imu = imu
        self.ticks_per_mm = ticks_per_mm
        self.x = self.y = 0.0
        self._lock = threading.Lock()
        _, self._l, self._r = rover.encoders()
        self.last = (0, self._l, self._r, 0.0, 0.0, imu.yaw)

    def update(self) -> tuple[int, int, int, float, float, float]:
        """(ms, left, right, x_mm, y_mm, yaw) after integrating since the last call."""
        with self._lock:
            ms, left, right = self.rover.encoders()
            yaw = self.imu.yaw
            ds = ((left - self._l) + (right - self._r)) / 2 / self.ticks_per_mm
            self._l, self._r = left, right
            self.x += ds * math.sin(math.radians(yaw))
            self.y += ds * math.cos(math.radians(yaw))
            self.last = (ms, left, right, round(self.x, 1), round(self.y, 1), round(yaw, 2))
            return self.last


# ---- run ----

class SurveyAborted(RuntimeError):
    pass


@dataclass
class SurveyLog:
    folder: Path
    events: list[dict] = field(default_factory=list)

    def add(self, kind: str, **data) -> None:
        e = {"t": round(time.time(), 3), "kind": kind, **data}
        self.events.append(e)
        with open(self.folder / "events.jsonl", "a") as f:
            f.write(json.dumps(e) + "\n")


class SurveyRunner:
    """Drives a Plan. rover: Rover-like, imu: has .yaw, heading: HeadingControl.
    camera (optional): rover.camera.Camera, recorded at settings.save_fps.
    guard (optional): rover.power.PowerGuard, checked between moves."""

    def __init__(self, plan: Plan, rover, imu, heading, folder: Path, camera=None, guard=None):
        self.plan = plan
        self.rover = rover
        self.imu = imu
        self.heading = heading
        self.folder = folder
        self.camera = camera
        self.guard = guard
        self.log = SurveyLog(folder)
        self.odo = Odometer(rover, imu, rover.config.ticks_per_mm)
        self.phase = "start"
        self.lane = -1

    def run(self) -> dict:
        p, s = self.plan, self.plan.settings
        (self.folder / "plan.json").write_text(json.dumps(asdict(p), indent=2))
        recorder = None
        if self.camera is not None:
            from rover.capture import Recorder
            recorder = Recorder(self.camera, self.folder, s.save_fps,
                                extra_fields=["ms", "enc_left", "enc_right", "x_mm", "y_mm", "yaw", "phase", "lane"],
                                extra=lambda: [*self.odo.update(), self.phase, self.lane]).start()
        status, started = "complete", time.time()
        try:
            for lane in p.lanes:
                if lane.index > 0:
                    self._shift_to(lane)
                self._drive_lane(lane)
        except SurveyAborted as e:
            status = f"aborted: {e}"
        except BaseException as e:
            status = f"error: {type(e).__name__}: {e}"
            raise
        finally:
            self.phase = "end"
            try:
                self.rover.stop()
            finally:
                frames = recorder.stop() if recorder else 0
                summary = {
                    "status": status,
                    "seconds": round(time.time() - started, 1),
                    "lanes_planned": len(p.lanes),
                    "lanes_done": sum(1 for e in self.log.events if e["kind"] == "lane_done"),
                    "frames": frames,
                    "end_pose": self.odo.update()[3:],
                    "plan": p.describe(),
                }
                (self.folder / "summary.json").write_text(json.dumps(summary, indent=2))
        return summary

    def _check(self) -> None:
        if self.guard is not None and self.guard.low.is_set():
            raise SurveyAborted("; ".join(self.guard.reasons) or "battery low")

    def _turn(self, target: float) -> None:
        self._check()
        rep = self.heading.turn_to(target, None)
        self.log.add("turn", target=target, final=round(rep.final, 2), steps=rep.steps)

    def _drive(self, mm: float, heading: float) -> None:
        self._check()
        errs = self.heading.drive_straight(mm, heading, segment_mm=abs(mm) or 1, speed=self.plan.settings.speed)
        self.log.add("drive", mm=round(mm, 1), heading=heading, heading_error=errs[-1] if errs else None,
                     pose=self.odo.update()[3:])

    def _pose(self) -> tuple[float, float]:
        _, _, _, x, y, _ = self.odo.update()
        return x, y

    def _drive_lane(self, lane: Lane) -> None:
        """Drive to the lane's end in segments, steering back onto its line."""
        self.lane, self.phase = lane.index, "lane"
        self.log.add("lane_start", lane=lane.index, pose=self.odo.update()[3:])
        x_line, _, y_end = self.plan.track(lane)
        forward = 1 if lane.index % 2 == 0 else -1     # lane runs toward +y or -y
        seg = self.plan.settings.segment_mm
        while True:
            x, y = self._pose()
            remaining = (y_end - y) * forward
            if remaining <= MIN_MOVE_MM:
                break
            step = min(seg, remaining)
            # Aim back at the line over the next couple of segments.
            off = x - x_line
            correction = -forward * math.degrees(math.atan2(off, LOOKAHEAD_SEGMENTS * seg))
            correction = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, correction))
            self._drive(step, lane.heading + correction)
        self.log.add("lane_done", lane=lane.index, pose=self.odo.update()[3:])

    def _shift_to(self, lane: Lane) -> None:
        """Finish the previous lane's end, move across, face the new lane."""
        p = self.plan
        prev = p.lanes[lane.index - 1]
        x_new, y_new, _ = p.track(lane)
        prev_forward = 1 if prev.index % 2 == 0 else -1
        self.phase = "transit"
        _, y = self._pose()
        extra = (y_new - y) * prev_forward          # camera-ahead allowance (+ any shortfall)
        if abs(extra) > MIN_MOVE_MM:
            self._drive(extra, prev.heading)
        self.phase = "turn"
        self._turn(p.shift_heading)
        self.phase = "transit"
        x, _ = self._pose()
        sign = 1 if p.settings.side == "right" else -1
        self._drive((x_new - x) * sign, p.shift_heading)
        self.phase = "turn"
        self._turn(lane.heading)


def new_survey_dir(label: str = "") -> Path:
    name = time.strftime("%Y%m%d-%H%M%S") + (f"-{label}" if label else "")
    path = SURVEY_DIR / name
    path.mkdir(parents=True)
    return path
