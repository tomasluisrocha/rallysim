import unittest

import numpy as np
import pandas as pd

from rallytools.channels import derived
from rallytools.run import from_frame
from rallytools.suspension import steady, table, window


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
        self.assertIn("not steady", table([run]))

    def test_non_four_wheel_rejected(self):
        run = from_frame(pd.DataFrame({"time": [0.0], "yaw": [0.0], "speed": [1.0]}),
                         kind="bicycle")
        with self.assertRaises(ValueError):
            derived(run, "yaw.unwrapped")


if __name__ == "__main__":
    unittest.main()
