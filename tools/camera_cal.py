#!/usr/bin/env python3
"""Camera calibration with the printed checkerboard (motors never move).

    python3 tools/camera_cal.py aim                  board flat under the camera:
                                                     height, tilt, autofocus value
    python3 tools/camera_cal.py snap  --session S    save one view for the lens fit
    python3 tools/camera_cal.py solve --session S    fit the lens model -> config
    python3 tools/camera_cal.py solve --session S --height-mm H
                                                     same from flat floor views only, with the
                                                     lens-to-floor height measured by tape
    python3 tools/camera_cal.py ground               board flat on the floor:
                                                     height, tilt, mm/px, footprint -> config

Board: docs/checkerboard_25mm.pdf; pass --square-mm if the print isn't 25.0.
Lens views: 15-20 snaps, board covering every part of the frame, some flat and
at least 5 propped up 20-40 deg on each side (solve refuses without them), on
a rigid backing. Or: flat views only plus a tape-measured height (--height-mm);
tilted views are then ignored. Focus must be fixed ([camera] focus)
before snap/solve/ground, since the lens model changes with focus.
Files: data/calib/<session>/NNN.jpg (gitignored); result config/camera_calibration.json.
"""

from __future__ import annotations

import argparse
import dataclasses
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rover.camera import Camera, load_camera_config  # noqa: E402
from rover.camera_model import (  # noqa: E402
    CALIBRATION_PATH, PATTERN, CameraModel, approx_model, board_tilt_deg, calibrate, find_board,
    focal_from_height, ground_from_board,
)

# Flat-only views can't separate focal length from distance (2026-11-27: 19 flat
# views fitted a 40 deg lens at 52 cm for a 70 deg lens at 26 cm). Demand tilted ones.
TILTED_DEG = 12
MIN_TILTED = 5
FLAT_DEG = 5               # --height-mm uses only views lying flat on the floor
CALIB_DIR = Path(__file__).resolve().parent.parent / "data" / "calib"
# The board sits still, so a longer exposure than the driving cap is fine (less gain noise).
STILL_EXPOSURE_MAX = 300


def grab(require_fixed_focus: bool = True) -> tuple[np.ndarray, Camera]:
    cfg = load_camera_config()
    if require_fixed_focus and cfg.focus is None:
        sys.exit("Set [camera] focus in config/rover.toml first (run `aim` to see the autofocus value).")
    cam = Camera(dataclasses.replace(cfg, exposure_max=STILL_EXPOSURE_MAX)).open()
    try:
        _, frame = cam.read()
    finally:
        cam.close()
    return frame, cam


def describe_ground(g, label: str) -> None:
    def lean(v: float, pos: str, neg: str) -> str:
        return "level" if abs(v) < 0.5 else f"{abs(v):.1f} deg toward image {pos if v > 0 else neg}"
    print(f"{label}: height {g.height_mm:.0f} mm")
    print(f"  tilt: {lean(g.lean_top_deg, 'top', 'bottom')}, {lean(g.lean_right_deg, 'right', 'left')}")
    print(f"  ground per frame: {g.footprint_w_mm:.0f} x {g.footprint_h_mm:.0f} mm, "
          f"{g.mm_per_px:.3f} mm/px at the centre")


def annotate(frame: np.ndarray, corners: np.ndarray | None, path: Path) -> None:
    out = frame.copy()
    if corners is not None:
        cv2.drawChessboardCorners(out, PATTERN, corners, True)
    h, w = out.shape[:2]
    cv2.line(out, (w // 2, 0), (w // 2, h), (0, 255, 255), 1)
    cv2.line(out, (0, h // 2), (w, h // 2), (0, 255, 255), 1)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), out, [cv2.IMWRITE_JPEG_QUALITY, 85])


def model_or_approx(frame: np.ndarray) -> tuple[CameraModel, bool]:
    if CALIBRATION_PATH.exists():
        return CameraModel.load(), True
    h, w = frame.shape[:2]
    return approx_model(w, h), False


def cmd_aim(args) -> None:
    frame, cam = grab(require_fixed_focus=False)
    s = cam.settings
    print(f"autofocus chose focus={s.focus} (exposure {s.exposure}, gain {s.gain})")
    corners = find_board(frame)
    annotate(frame, corners, CALIB_DIR / "aim.jpg")
    print(f"image: {CALIB_DIR / 'aim.jpg'}")
    if corners is None:
        sys.exit("board not found: is it fully in view, flat and lit evenly?")
    model, calibrated = model_or_approx(frame)
    g = ground_from_board(model, corners, args.square_mm)
    describe_ground(g, "pose" + ("" if calibrated else " (rough: lens not calibrated yet)"))


def cmd_snap(args) -> None:
    folder = CALIB_DIR / args.session
    folder.mkdir(parents=True, exist_ok=True)
    frame, _ = grab()
    corners = find_board(frame)
    if corners is None:
        annotate(frame, None, folder / "last_miss.jpg")
        sys.exit(f"board NOT found (frame in {folder / 'last_miss.jpg'}); move it fully into view")
    n = len(list(folder.glob("[0-9]*.jpg"))) + 1
    cv2.imwrite(str(folder / f"{n:03d}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    print(f"view {n} saved")
    print_coverage(folder)


def load_views(folder: Path) -> tuple[list[np.ndarray], list[Path], tuple[int, int]]:
    sets, paths, size = [], [], None
    for p in sorted(folder.glob("[0-9]*.jpg")):
        img = cv2.imread(str(p))
        size = (img.shape[1], img.shape[0])
        c = find_board(img)
        if c is not None:
            sets.append(c)
            paths.append(p)
    return sets, paths, size


def print_coverage(folder: Path) -> None:
    """Which parts of the frame have had board corners (4 x 3 grid)."""
    sets, _, size = load_views(folder)
    if not sets:
        return
    grid = np.zeros((3, 4), int)
    for c in sets:
        for x, y in c.reshape(-1, 2):
            grid[min(2, int(y / size[1] * 3)), min(3, int(x / size[0] * 4))] += 1
    print("coverage (corners per area of the frame; aim for every cell > 0):")
    for row in grid:
        print("  " + " ".join(f"{v:4d}" for v in row))


def cmd_solve(args) -> None:
    folder = CALIB_DIR / args.session
    sets, paths, size = load_views(folder)
    focal = None
    if args.height_mm:
        rough = approx_model(*size)
        flat = [i for i, c in enumerate(sets) if board_tilt_deg(rough, c, args.square_mm) < FLAT_DEG]
        if len(flat) < len(sets):
            print("ignoring tilted views:", ", ".join(paths[i].name for i in range(len(sets)) if i not in flat))
        sets, paths = [sets[i] for i in flat], [paths[i] for i in flat]
    if len(sets) < 8:
        sys.exit(f"only {len(sets)} usable views in {folder}; take at least 8 (15-20 is better)")
    if args.height_mm:
        focal = focal_from_height(sets, size, args.height_mm, args.square_mm)
    model, per_view = calibrate(sets, size, args.square_mm, focal)
    # Drop clear outliers (a smudged or moved view) and refit once.
    med = float(np.median(per_view))
    keep = [i for i, e in enumerate(per_view) if e <= max(3 * med, 0.5)]
    if len(keep) < len(sets):
        print("dropping views:", ", ".join(paths[i].name for i in range(len(sets)) if i not in keep))
        sets = [sets[i] for i in keep]
        if args.height_mm:
            focal = focal_from_height(sets, size, args.height_mm, args.square_mm)
        model, per_view = calibrate(sets, size, args.square_mm, focal)
    tilts = [board_tilt_deg(model, c, args.square_mm) for c in sets]
    n_tilted = sum(t >= TILTED_DEG for t in tilts)
    print(f"board tilt per view (deg): {', '.join(f'{t:.0f}' for t in tilts)}")
    if not args.height_mm and n_tilted < MIN_TILTED:
        sys.exit(f"only {n_tilted} views tilted >= {TILTED_DEG} deg; need {MIN_TILTED}+ (prop one edge "
                 f"of the board up 20-30 deg, each side). Not saved.")
    model.focus = load_camera_config().focus
    model.notes = {"session": args.session, "square_mm": args.square_mm,
                   "method": f"flat views, focal from measured height {args.height_mm} mm" if args.height_mm
                   else "flat + tilted views"}
    if CALIBRATION_PATH.exists():
        model.ground = None  # the old ground pose used the old lens model: redo `ground`
    model.save()
    k = model.K
    print(f"views used {model.views}, RMS error {model.rms_px:.3f} px (good: < 0.5)")
    print(f"focal length {k[0][0]:.1f} / {k[1][1]:.1f} px, centre ({k[0][2]:.1f}, {k[1][2]:.1f})")
    print(f"distortion {np.round(model.dist, 4).tolist()}")
    print(f"saved {CALIBRATION_PATH}; now run `ground` with the board flat on the floor")


def cmd_ground(args) -> None:
    if not CALIBRATION_PATH.exists():
        sys.exit("calibrate the lens first (snap + solve)")
    frame, _ = grab()
    corners = find_board(frame)
    annotate(frame, corners, CALIB_DIR / "ground.jpg")
    if corners is None:
        sys.exit(f"board not found ({CALIB_DIR / 'ground.jpg'})")
    model = CameraModel.load()
    model.ground = ground_from_board(model, corners, args.square_mm)
    model.save()
    describe_ground(model.ground, "ground")
    print(f"saved {CALIBRATION_PATH}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--square-mm", type=float, default=25.0)
    sub = parser.add_subparsers(dest="cmd", required=True)
    sub.add_parser("aim")
    for name in ("snap", "solve"):
        p = sub.add_parser(name)
        p.add_argument("--session", default="lens")
        if name == "solve":
            p.add_argument("--height-mm", type=float, help="tape-measured lens-to-floor height; "
                           "fits from flat floor views alone")
    sub.add_parser("ground")
    args = parser.parse_args()
    {"aim": cmd_aim, "snap": cmd_snap, "solve": cmd_solve, "ground": cmd_ground}[args.cmd](args)


if __name__ == "__main__":
    main()
