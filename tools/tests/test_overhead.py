import subprocess
import sys
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

from rallytools import overhead as O
from rallytools import run

FIXTURES = Path(__file__).parent / "fixtures"
TOOLS = Path(__file__).resolve().parents[1]
WHEELS = ("FL", "FR", "RL", "RR")


def four_wheel(rows, meta=None, **wheel):
    """A four-wheel run of len(rows) rows; `rows` holds per-row dicts of the columns that vary."""
    n = len(rows)
    d = {"time": np.arange(n) * 0.1, "pos.x": np.zeros(n), "pos.y": np.zeros(n), "yaw": np.zeros(n),
         "speed": np.zeros(n), "steer": np.zeros(n), "yaw_rate": np.zeros(n), "throttle": np.zeros(n),
         "brake": np.zeros(n), "handbrake": np.zeros(n)}
    for w in WHEELS:
        d.update({f"wheel.{w}.Fx": np.zeros(n), f"wheel.{w}.Fy": np.zeros(n), f"wheel.{w}.Fz": np.full(n, 3000.0),
                  f"wheel.{w}.omega": np.full(n, 30.0), f"wheel.{w}.slipRatio": np.zeros(n)})
    for k, row in enumerate(rows):
        for key, value in row.items():
            if key in d:
                d[key][k] = value
            else:
                for w in WHEELS:
                    d[f"wheel.{w}.{key}"][k] = value
    return run.from_frame(pd.DataFrame(d), meta=meta)


class TestOverhead(unittest.TestCase):
    def test_four_wheel_geometry(self):
        meta = {"lf": 1.3, "lr": 1.2, "track_front": 1.5, "track_rear": 1.4}
        v = O.view_of(four_wheel([{}, {}], meta=meta))
        got = {w.label: (w.x, w.y) for w in v.wheels}
        want = {"FL": (1.3, 0.75), "FR": (1.3, -0.75), "RL": (-1.2, 0.7), "RR": (-1.2, -0.7)}
        for label, (x, y) in want.items():
            self.assertAlmostEqual(got[label][0], x)
            self.assertAlmostEqual(got[label][1], y)
        steer = np.array([0.1, 0.2])
        v = O.view_of(four_wheel([{"steer": 0.1}, {"steer": 0.2}], meta=meta))
        for w in v.wheels:
            np.testing.assert_array_equal(w.steer, steer if w.label[0] == "F" else np.zeros(2))
        self.assertAlmostEqual(v.body_length, 1.3 + 1.2 + 1.6)

    def test_defaults(self):
        v = O.view_of(four_wheel([{}, {}]))
        d = O.CAR_DEFAULTS
        self.assertAlmostEqual(v.wheels[0].x, d["lf"])
        self.assertAlmostEqual(v.wheels[0].y, d["track_front"] / 2)
        self.assertAlmostEqual(v.wheels[3].x, -d["lr"])
        self.assertAlmostEqual(v.wheels[3].y, -d["track_rear"] / 2)
        self.assertAlmostEqual(v.tyre_length, 2 * d["wheel_radius"])
        self.assertEqual(O.with_car_defaults({"lf": 2.0})["lf"], 2.0)
        self.assertEqual(O.with_car_defaults({"lf": 2.0})["lr"], d["lr"])

    def test_kinds(self):
        for name, kind in (("rigid_body", "rigid body"), ("bicycle", "kinematic bicycle"),
                           ("single_track", "single-track"), ("four_wheel_springs", "four-wheel car"),
                           ("four_wheel_schema5", "four-wheel car"), ("four_wheel_unsprung", "four-wheel car")):
            with self.subTest(name=name):
                self.assertEqual(O.view_of(run.load(FIXTURES / f"{name}.csv")).kind, kind)

    def test_path_is_run_name(self):
        r = run.load(FIXTURES / "bicycle.csv")
        self.assertEqual(O.view_of(r).path, "bicycle")

    def test_wheel_state(self):
        v = O.view_of(four_wheel([{"Fz": 0},                       # off the ground
                                  {"omega": 0, "speed": 10},       # stopped while the car moves
                                  {"speed": 10, "slipRatio": 0.3},
                                  {"speed": 10, "slipRatio": -0.3},
                                  {"omega": 0, "slipRatio": 0.3}]))  # standstill: slip means nothing
        w = v.wheels[0]
        self.assertEqual([O.wheel_state(v, w, i) for i in range(5)], ["AIR", "LOCK", "SPIN", "SLIDE", ""])

    def test_force_scale(self):
        n = 2
        d = {"time": [0.0, 0.1], "pos.x": [0.0] * n, "pos.y": [0.0] * n, "yaw": [0.0] * n, "speed": [5.0] * n,
             "steer": [0.0] * n, "yaw_rate": [0.0] * n, "axle.front.Fy": [1000.0] * n,
             "axle.rear.Fy": [1000.0] * n, "drive_force": [0.0] * n}
        v = O.view_of(run.from_frame(pd.DataFrame(d)))
        self.assertAlmostEqual(O.force_scale(v, totals=False), O.FORCE_ARROW_MAX / 1000)  # largest tyre force
        self.assertAlmostEqual(O.force_scale(v), O.NET_ARROW_MAX / 2000)                  # the sum is the limit
        v.wheels = []
        v.net_force = None
        self.assertEqual(O.force_scale(v), 0.0)

    def test_row_at(self):
        v = O.view_of(four_wheel([{}, {}, {}]))
        self.assertEqual([O.row_at(v, t) for t in (-1, 0, 0.05, 0.1, 0.15, 9)], [0, 0, 1, 1, 2, 2])

    def test_sideslip(self):
        d = {"time": [0.0, 0.1, 0.2], "pos.x": [0.0] * 3, "pos.y": [0.0] * 3, "yaw": [0.0] * 3,
             "speed": [0.1, 0.3, 5.0], "beta": [0.2] * 3, "steer": [0.0] * 3, "yaw_rate": [0.0] * 3}
        beta = O.view_of(run.from_frame(pd.DataFrame(d))).beta
        self.assertTrue(np.isnan(beta[0]) and np.isnan(beta[1]))  # at or below 0.3 m/s
        self.assertEqual(beta[2], 0.2)

    def test_unknown_frame(self):
        r = run.Run("x", "bicycle", None, None, {}, {}, pd.DataFrame({"time": [0.0], "foo": [1.0]}))
        with self.assertRaises(ValueError):
            O.view_of(r)

    def test_no_matplotlib(self):
        code = "import sys, rallytools.overhead; sys.exit('matplotlib' in sys.modules)"
        done = subprocess.run([sys.executable, "-c", code], cwd=TOOLS)
        self.assertEqual(done.returncode, 0)


if __name__ == "__main__":
    unittest.main()
