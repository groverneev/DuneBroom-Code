#!/usr/bin/env python3
"""Plan and run a plot survey (rover/survey.py).

    python3 tools/survey.py plan --width 3 --length 5      lanes, coverage, time (metres)
    python3 tools/survey.py run  --width 3 --length 5 --dry-run
                                                           simulated rover, no hardware
    python3 tools/survey.py run  --width 3 --length 5      MOVES MOTORS: the real survey

Start position: rover at the plot's near corner, facing along the plot, with
the lanes advancing to the side set in config/rover.toml [survey] side.
Output: data/surveys/<date-time>[-label]/ with plan.json, events.jsonl,
frames/ + frames.csv (pose, heading, phase, lane per frame), summary.json.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from rover.survey import SurveyRunner, load_survey_settings, make_plan, new_survey_dir  # noqa: E402


def build_plan(args):
    return make_plan(args.width * 1000, args.length * 1000, load_survey_settings())


def run_dry(args, plan) -> None:
    import rover.heading as heading_mod
    from rover.heading import HeadingControl
    from rover.sim import SimRover
    heading_mod.SETTLE_S = 0
    sim = SimRover(seed=args.seed)
    folder = new_survey_dir(f"{args.label}-dry" if args.label else "dry")
    summary = SurveyRunner(plan, sim, sim, HeadingControl(sim, sim), folder).run()
    ox, oy, _ = summary["end_pose"]
    print(f"{summary['status']}: {summary['lanes_done']}/{summary['lanes_planned']} lanes, {sim.moves} moves")
    print(f"end: odometry ({ox:.0f}, {oy:.0f}) mm, true ({sim.x:.0f}, {sim.y:.0f}) mm, yaw {sim.yaw:.1f} deg")
    print(f"saved {folder}")


def run_real(args, plan) -> None:
    from rover import Rover
    from rover.camera import Camera
    from rover.heading import HeadingControl
    from rover.imu import BNO055
    from rover.power import PowerGuard

    with Rover() as rover, BNO055() as imu:
        guard = PowerGuard(rover)
        problems = guard.check_start()
        if problems:
            sys.exit("not starting: " + "; ".join(problems))
        imu.start_tracking()
        deadline = time.monotonic() + 15
        while imu.calibration()["gyro"] < 3:   # gyro calibrates while standing still
            if time.monotonic() > deadline:
                sys.exit("gyro did not calibrate; keep the rover still and retry")
            time.sleep(0.2)
        camera = None if args.no_camera else Camera().open()
        try:
            folder = new_survey_dir(args.label)
            guard.start()
            runner = SurveyRunner(plan, rover, imu, HeadingControl(rover, imu), folder,
                                  camera=camera, guard=guard)
            summary = runner.run()
        finally:
            guard.stop()
            if camera:
                camera.close()
    print(f"{summary['status']}: {summary['lanes_done']}/{summary['lanes_planned']} lanes, "
          f"{summary['frames']} frames, {summary['seconds']:.0f} s")
    print(f"saved {folder}")


def main() -> None:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    for name in ("plan", "run"):
        p = sub.add_parser(name)
        p.add_argument("--width", type=float, required=True, help="metres across the lanes")
        p.add_argument("--length", type=float, required=True, help="metres along the lanes")
        if name == "run":
            p.add_argument("--dry-run", action="store_true")
            p.add_argument("--seed", type=int, default=0, help="dry run randomness")
            p.add_argument("--no-camera", action="store_true")
            p.add_argument("--label", default="")
    args = parser.parse_args()
    plan = build_plan(args)
    if args.cmd == "plan":
        print(plan.describe())
        return
    print(plan.describe())
    (run_dry if args.dry_run else run_real)(args, plan)


if __name__ == "__main__":
    main()
