"""Camera geometry: lens calibration and where the image lands on the ground.

Calibration uses the printed checkerboard (tools/make_checkerboard.py): many
views give the lens model (focal length, centre, distortion); one view of the
board lying flat on the ground gives the camera's height and tilt, mm per
pixel and the ground footprint of a frame. Saved as JSON in
config/camera_calibration.json; everything downstream reads it from there.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import cv2
import numpy as np

CALIBRATION_PATH = Path(__file__).resolve().parent.parent / "config" / "camera_calibration.json"
PATTERN = (6, 8)          # inner corners of the 7 x 9 board
SQUARE_MM = 25.0


@dataclass
class Ground:
    """The camera's pose over a flat floor, from one board view."""
    height_mm: float
    lean_top_deg: float     # + = camera looks toward the image's top edge
    lean_right_deg: float   # + = camera looks toward the image's right edge
    mm_per_px: float        # at the image centre
    footprint_mm: list[list[float]]   # image corners TL, TR, BR, BL on the ground (board frame)
    footprint_w_mm: float   # mean of top/bottom edge lengths
    footprint_h_mm: float   # mean of left/right edge lengths


@dataclass
class CameraModel:
    width: int
    height: int
    K: list[list[float]]
    dist: list[float]
    rms_px: float
    views: int
    focus: int | None = None
    ground: Ground | None = None
    notes: dict = field(default_factory=dict)

    @property
    def K_np(self) -> np.ndarray:
        return np.array(self.K, dtype=np.float64)

    @property
    def dist_np(self) -> np.ndarray:
        return np.array(self.dist, dtype=np.float64)

    def save(self, path: Path = CALIBRATION_PATH) -> None:
        path.write_text(json.dumps(asdict(self), indent=2) + "\n")

    @classmethod
    def load(cls, path: Path = CALIBRATION_PATH) -> "CameraModel":
        d = json.loads(Path(path).read_text())
        g = d.pop("ground", None)
        return cls(**d, ground=Ground(**g) if g else None)

    def undistort(self, image: np.ndarray) -> np.ndarray:
        return cv2.undistort(image, self.K_np, self.dist_np)


def approx_model(width: int, height: int) -> CameraModel:
    """C920 at 16:9, ~70 deg horizontal view, no distortion: good enough to aim."""
    f = width / 2 / np.tan(np.radians(70.4 / 2))
    K = [[f, 0, (width - 1) / 2], [0, f, (height - 1) / 2], [0, 0, 1]]
    return CameraModel(width, height, K, [0.0] * 5, rms_px=float("nan"), views=0)


def board_points(square_mm: float = SQUARE_MM) -> np.ndarray:
    cols, rows = PATTERN
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    return np.hstack([grid, np.zeros((len(grid), 1))]).astype(np.float32)


def find_board(image: np.ndarray) -> np.ndarray | None:
    """Inner corners (N x 1 x 2, sub-pixel) or None."""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCornersSB(gray, PATTERN, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
    return corners.astype(np.float32) if ok else None


def calibrate(corner_sets: list[np.ndarray], size: tuple[int, int],
              square_mm: float = SQUARE_MM) -> tuple[CameraModel, list[float]]:
    """Lens model from board views; also each view's RMS error (px)."""
    obj = board_points(square_mm)
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        [obj] * len(corner_sets), corner_sets, size, None, None, flags=cv2.CALIB_FIX_K3)
    per_view = []
    for c, r, t in zip(corner_sets, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, r, t, K, dist)
        per_view.append(float(np.sqrt(np.mean(np.sum((proj - c) ** 2, axis=2)))))
    model = CameraModel(size[0], size[1], K.tolist(), dist.ravel().tolist(), float(rms), len(corner_sets))
    return model, per_view


def ground_from_board(model: CameraModel, corners: np.ndarray, square_mm: float = SQUARE_MM) -> Ground:
    """Camera pose over the plane the board lies on."""
    ok, rvec, tvec = cv2.solvePnP(board_points(square_mm), corners, model.K_np, model.dist_np)
    if not ok:
        raise ValueError("solvePnP failed")
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.ravel()
    n = R[:, 2]                      # board normal in camera coordinates
    if n @ t > 0:                    # make it point from the ground toward the camera
        n = -n
    h = -(n @ t)                     # camera distance to the plane
    lean_top = np.degrees(np.arctan2(-n[1], -n[2]))
    lean_right = np.degrees(np.arctan2(n[0], -n[2]))

    def to_ground(px: np.ndarray) -> np.ndarray:
        """Pixels -> points on the plane, in board coordinates (mm)."""
        norm = cv2.undistortPoints(px.reshape(-1, 1, 2).astype(np.float64), model.K_np, model.dist_np)
        rays = np.hstack([norm.reshape(-1, 2), np.ones((len(norm), 1))])
        pts = rays * ((n @ t) / (rays @ n))[:, None]
        return (R.T @ (pts - t).T).T[:, :2]

    w, hgt = model.width, model.height
    corners_img = np.array([[0, 0], [w - 1, 0], [w - 1, hgt - 1], [0, hgt - 1]], dtype=np.float64)
    fp = to_ground(corners_img)
    edge = lambda a, b: float(np.linalg.norm(fp[a] - fp[b]))
    c = np.array([[w / 2, hgt / 2], [w / 2 + 1, hgt / 2], [w / 2, hgt / 2 + 1]])
    g = to_ground(c)
    mm_per_px = (np.linalg.norm(g[1] - g[0]) + np.linalg.norm(g[2] - g[0])) / 2
    return Ground(
        height_mm=round(float(h), 1),
        lean_top_deg=round(float(lean_top), 2),
        lean_right_deg=round(float(lean_right), 2),
        mm_per_px=round(float(mm_per_px), 4),
        footprint_mm=np.round(fp, 1).tolist(),
        footprint_w_mm=round((edge(0, 1) + edge(3, 2)) / 2, 1),
        footprint_h_mm=round((edge(0, 3) + edge(1, 2)) / 2, 1),
    )
