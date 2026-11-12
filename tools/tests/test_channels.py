import contextlib
import io
from pathlib import Path
import tempfile
import unittest

import numpy as np
import pandas as pd

from rallytools import cli
from rallytools.channels import derived
from rallytools.run import from_frame
from rallytools.suspension import steady, table, window
from .support import write_run


def four(rows=3, name="run", **updates):
    data = {
        "time": np.arange(rows, dtype=float) * 0.001,
        "roll": np.full(rows, 0.02),
        "a_lat": np.full(rows, 4.0),
        "yaw": np.zeros(rows),
    }
    for wheel, fz in (("FL", 2000.0), ("FR", 4000.0), ("RL", 2500.0), ("RR", 3500.0)):
        data[f"wheel.{wheel}.Fz"] = np.full(rows, fz)
        data[f"wheel.{wheel}.Fx"] = np.full(rows, 30.0)
        data[f"wheel.{wheel}.Fy"] = np.full(rows, 40.0)
        data[f"wheel.{wheel}.grip"] = np.full(rows, 100.0)
        data[f"wheel.{wheel}.deflection"] = np.full(rows, 0.1)
    data.update(updates)
    return from_frame(pd.DataFrame(data), kind="four_wheel", name=name)


class TestChannels(unittest.TestCase):
    def test_load_transfer_and_total(self):
        run = four()
        self.assertTrue(np.all(derived(run, "transfer.front") == 1000))
        self.assertTrue(np.all(derived(run, "transfer.rear") == 500))
        np.testing.assert_allclose(derived(run, "transfer.front_share"), 2 / 3, atol=1e-12)
        self.assertTrue(np.all(derived(run, "load.total") == 12000))

    def test_roll_gradient_is_degrees(self):
        self.assertAlmostEqual(steady(four(), 0, 0.002)["roll_gradient"], 0.2864788975654116, places=12)

    def test_default_window_includes_both_ends(self):
        values = np.zeros(2001)
        values[1000:] = 1.0
        run = four(2001, a_lat=values)
        self.assertEqual(np.mean(run.column("a_lat")[window(run)]), 1.0)
        self.assertEqual(int(window(run).sum()), 1001)

    def test_grip_use_zero_grip_is_nan(self):
        run = four()
        run.data.loc[1, "wheel.FL.grip"] = 0.0
        run.data.loc[1, "wheel.FL.Fx"] = 100.0
        values = derived(run, "wheel.FL.gripUse")
        self.assertTrue(np.isnan(values[1]))
        self.assertTrue(np.isfinite(values[[0, 2]]).all())

    def test_front_share_zero_sum_is_nan(self):
        run = four()
        run.data["wheel.FL.Fz"] = 4000.0
        run.data["wheel.FR.Fz"] = 4600.0
        run.data["wheel.RL.Fz"] = 3100.0
        run.data["wheel.RR.Fz"] = 2500.0
        self.assertTrue(np.isnan(derived(run, "transfer.front_share")).all())

    def test_unsteady_ramp_is_marked(self):
        roll = np.linspace(0.01, 0.02, 3)
        run = four(roll=roll)
        self.assertFalse(steady(run)["is_steady"])
        lines = dict(line.split("\t", 1) for line in table([run]).splitlines()[1:])
        for quantity in ("roll_deg", "a_lat", "roll_gradient", "transfer_front", "transfer_rear",
                         "front_share"):
            self.assertEqual(lines[quantity], "not steady")
        # Whole-run values do not depend on the window: still shown (roll peaks at 0.02 rad).
        self.assertAlmostEqual(float(lines["max_abs_roll_deg"]), np.degrees(0.02), places=12)
        self.assertEqual(float(lines["min_fz"]), 2000.0)
        self.assertEqual(float(lines["max_deflection"]), 0.1)

    def stepped(self):
        """3001 rows: one set of values before 2.0 s, the usual ones from 2.0 s on (the default window)."""
        early = np.arange(3001) < 2000
        columns = {"roll": np.where(early, 0.01, 0.02), "a_lat": np.where(early, 2.0, 4.0)}
        for wheel, fz in (("FL", 2000.0), ("FR", 4000.0), ("RL", 2500.0), ("RR", 3500.0)):
            columns[f"wheel.{wheel}.Fz"] = np.where(early, 3000.0, fz)
        return four(3001, **columns)

    def test_window_means(self):
        got = steady(self.stepped())
        self.assertTrue(got["is_steady"])
        self.assertAlmostEqual(got["roll_deg"], np.degrees(0.02), places=12)
        self.assertEqual(got["a_lat"], 4.0)
        self.assertEqual(got["transfer_front"], 1000.0)
        self.assertEqual(got["transfer_rear"], 500.0)
        self.assertAlmostEqual(got["front_share"], 2 / 3, places=12)

    def test_whole_run_values(self):
        # Each extreme sits on one early row, outside the window, on a wheel that is not the first.
        roll = np.full(3001, 0.02)
        roll[5] = -0.03
        fz = np.full(3001, 2500.0)
        fz[7] = 1500.0
        deflection = np.full(3001, 0.1)
        deflection[9] = 0.12
        got = steady(four(3001, roll=roll, **{"wheel.RL.Fz": fz, "wheel.RR.deflection": deflection}))
        self.assertAlmostEqual(got["max_abs_roll_deg"], np.degrees(0.03), places=12)
        self.assertEqual(got["min_fz"], 1500.0)
        self.assertEqual(got["max_deflection"], 0.12)

    def test_window_before_a_given_end(self):
        run = four(3001)
        selected = window(run, end=2.0)
        self.assertEqual(int(selected.sum()), 1001)
        self.assertEqual(run.column("time")[selected][0], 1.0)

    def test_empty_window_is_an_error(self):
        with self.assertRaises(ValueError):
            steady(four(3001), 2.0, 1.0)
        with self.assertRaises(ValueError):
            steady(four(3001), 5.0)

    def test_cli_window_options(self):
        data = self.stepped().data.assign(**{"wheel.FL.slipRatio": 0.0})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "stepped.csv"
            write_run(path, data)

            def a_lat(*options):
                output = io.StringIO()
                with contextlib.redirect_stdout(output), contextlib.redirect_stderr(io.StringIO()):
                    code = cli.main(["suspension", str(path), *options])
                rows = dict(line.split("\t", 1) for line in output.getvalue().splitlines())
                return code, rows.get("a_lat")

            self.assertEqual(a_lat(), (0, "4.0"))
            self.assertEqual(a_lat("--from", "0.5", "--to", "1.5"), (0, "2.0"))
            self.assertEqual(a_lat("--to", "1.5"), (0, "2.0"))
            self.assertEqual(a_lat("--from", "2", "--to", "1"), (2, None))

    def test_grip_use_and_unwrapped_yaw_values(self):
        run = four(yaw=np.array([3.0, -3.1, -2.9]))
        self.assertTrue(np.all(derived(run, "wheel.RL.gripUse") == 0.5))
        np.testing.assert_allclose(derived(run, "yaw.unwrapped"),
                                   [3.0, 2 * np.pi - 3.1, 2 * np.pi - 2.9], atol=1e-12)

    def test_non_four_wheel_rejected(self):
        run = from_frame(pd.DataFrame({"time": [0.0], "yaw": [0.0], "speed": [1.0]}),
                         kind="bicycle")
        with self.assertRaises(ValueError):
            derived(run, "yaw.unwrapped")


if __name__ == "__main__":
    unittest.main()
