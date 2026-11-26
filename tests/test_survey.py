#!/usr/bin/env python3
"""Survey planning and a simulated run.

    python3 tests/test_survey.py
"""

from __future__ import annotations

import csv
import dataclasses
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import rover.heading as heading_mod  # noqa: E402
from rover.capture import Recorder  # noqa: E402
from rover.heading import HeadingControl  # noqa: E402
from rover.sim import SimRover  # noqa: E402
from rover.survey import SurveyRunner, SurveySettings, make_plan  # noqa: E402

heading_mod.SETTLE_S = 0

S = SurveySettings(side="right", side_overlap=0.2, footprint_across_mm=670, footprint_along_mm=380,
                   camera_ahead_mm=0, speed=70, segment_mm=500, save_fps=5)


def test_plan_covers_plot():
    for width in (500, 670, 1000, 3000, 3333):
        p = make_plan(width, 5000, S)
        a = S.footprint_across_mm
        xs = [l.x_mm for l in p.lanes]
        assert xs[0] - a / 2 <= 0.01 or width <= a
        assert xs[-1] + a / 2 >= width - 0.01 or width <= a
        assert all(g <= a * (1 - S.side_overlap) + 0.01 for g in p.lane_gaps), (width, p.lane_gaps)
        assert abs(p.covered_area_m2() - width * 5000 / 1e6) < 0.01
    assert make_plan(3000, 5000, S).lanes[0].length_mm == 4620


def test_headings_and_tracks():
    p = make_plan(3000, 5000, S)
    assert [l.heading for l in p.lanes[:3]] == [0, 180, 0] and p.shift_heading == 90
    left = make_plan(3000, 5000, dataclasses.replace(S, side="left"))
    assert [l.heading for l in left.lanes[:2]] == [0, -180] and left.shift_heading == -90
    assert left.track(left.lanes[1])[0] < 0
    ahead = make_plan(3000, 5000, dataclasses.replace(S, camera_ahead_mm=150))
    assert ahead.track(ahead.lanes[1])[1:] == (4620 + 300, 300)


def run_sim(settings, seed, **sim_kw):
    plan = make_plan(3000, 5000, settings)
    sim = SimRover(seed=seed, **sim_kw)
    with tempfile.TemporaryDirectory() as d:
        summary = SurveyRunner(plan, sim, sim, HeadingControl(sim, sim), Path(d)).run()
    return plan, sim, summary


def test_sim_runs_end_on_the_last_lane():
    for side in ("right", "left"):
        for ahead in (0, 150):
            for seed in range(5):
                s = dataclasses.replace(S, side=side, camera_ahead_mm=ahead)
                plan, sim, summary = run_sim(s, seed, turn_scale=1.15)
                assert summary["status"] == "complete"
                x_end, _, y_end = plan.track(plan.lanes[-1])
                assert abs(sim.x - x_end) < 80, (side, ahead, seed, sim.x, x_end)
                assert abs(sim.y - y_end) < 150, (side, ahead, seed, sim.y, y_end)


def test_recorder_extra_columns():
    class FakeCam:
        def read(self):
            return time.time(), np.zeros((4, 4, 3), np.uint8)

        def encode_jpeg(self, frame):
            return b"jpg"

        def settings_dict(self):
            return {}

    with tempfile.TemporaryDirectory() as d:
        (Path(d) / "frames").mkdir()
        rec = Recorder(FakeCam(), Path(d), 50, extra_fields=["phase", "lane"], extra=lambda: ["lane", 3]).start()
        time.sleep(0.2)
        n = rec.stop()
        rows = list(csv.reader(open(Path(d) / "frames.csv")))
    assert n > 2 and rows[0][-2:] == ["phase", "lane"] and rows[1][-2:] == ["lane", "3"]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
