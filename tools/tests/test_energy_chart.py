import contextlib
import io
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.figure import Figure

from rallytools import cli, energy
from rallytools.charts.energy import figure
from rallytools.run import from_frame, load
from . import support

FIXTURES = Path(__file__).parent / "fixtures"
EXE = os.environ.get("RALLYSIM_EXE")


def built(residual=-1.0, name="built"):
    n = 11
    ramp = np.linspace(0.0, 1.0, n)
    total = 10.0 + 69.0 * ramp
    frame = pd.DataFrame({
        "time": np.linspace(0.0, 1.0, n), "wheel.FL.slipRatio": 0.0,
        "energy.translation": total - 10.0,
        "energy.rotation": 1.0, "energy.gravity": 2.0, "energy.springs": 3.0, "energy.wheels": 4.0,
        "energy.total": total,
        "work.drive": 100.0 * ramp, "work.brake": 30.0 * ramp, "work.damper": 0.0, "work.tyre": 0.0,
        "energy.residual": residual * ramp,
    })
    return from_frame(frame, kind="four_wheel", name=name)


class TestBalance(unittest.TestCase):
    def test_built_run_sums_and_gap(self):
        run = built()
        b = energy.balance(run)
        self.assertAlmostEqual(b["stored"][-1], 69.0, places=9)
        self.assertAlmostEqual(b["net_work"][-1], 70.0, places=9)
        self.assertTrue(np.allclose(b["gap"], 0.0, atol=1e-9))
        self.assertAlmostEqual(energy.balance(built(residual=0.0))["gap"][-1], -1.0, places=9)

    def test_every_work_takes_its_sign(self):
        # drive 100 in, brake 30, damper 7 and tyre 13 out: net 50; the store rises by 49, so the residual is -1
        n = 11
        ramp = np.linspace(0.0, 1.0, n)
        frame = pd.DataFrame({
            "time": ramp, "wheel.FL.slipRatio": 0.0,
            "energy.translation": 49.0 * ramp, "energy.rotation": 0.0, "energy.gravity": 0.0,
            "energy.springs": 0.0, "energy.wheels": 0.0, "energy.total": 49.0 * ramp,
            "work.drive": 100.0 * ramp, "work.brake": 30.0 * ramp, "work.damper": 7.0 * ramp,
            "work.tyre": 13.0 * ramp, "energy.residual": -1.0 * ramp,
        })
        b = energy.balance(from_frame(frame, kind="four_wheel", name="signs"))
        self.assertAlmostEqual(b["net_work"][-1], 50.0, places=9)
        self.assertTrue(np.allclose(b["gap"], 0.0, atol=1e-9))

    def test_fixtures_agree_with_their_books(self):
        for name in ("four_wheel_springs", "four_wheel_unsprung"):
            gap = energy.balance(load(FIXTURES / f"{name}.csv"))["gap"]
            self.assertLess(np.max(np.abs(gap)), 1e-6, name)

    def test_fresh_fw_launch(self):
        support.need(self, EXE and os.path.isfile(EXE), "RALLYSIM_EXE must name an existing rallysim executable")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "fw_launch.csv"
            result = subprocess.run([EXE, "fw_launch", str(output)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            gap = energy.balance(load(output))["gap"]
        self.assertLess(np.max(np.abs(gap)), 1e-6)

    def test_missing_columns_and_wrong_kind_raise(self):
        with self.assertRaises(ValueError):
            energy.balance(load(FIXTURES / "bicycle.csv"))
        with self.assertRaises(ValueError):
            energy.balance(from_frame(built().data.drop(columns="work.tyre"), kind="four_wheel"))


class TestChart(unittest.TestCase):
    def tearDown(self):
        plt.close("all")

    def test_axes_lines_and_end_values(self):
        chart = figure(built())
        self.assertEqual(len(chart.axes), 3)
        self.assertEqual([len(a.lines) for a in chart.axes], [5, 4, 3])
        self.assertEqual([a.get_ylabel() for a in chart.axes],
                         ["stored energy (J)", "work since the start (J)", "balance (J)"])
        lines = {line.get_label(): line for line in chart.axes[2].lines}
        self.assertEqual(set(lines), {"stored", "net work", "residual"})
        self.assertAlmostEqual(lines["stored"].get_ydata()[-1], 69.0, places=9)
        self.assertAlmostEqual(lines["net work"].get_ydata()[-1], 70.0, places=9)
        self.assertAlmostEqual(lines["residual"].get_ydata()[-1], -1.0, places=9)

    def test_draws_into_a_given_figure(self):
        target = Figure()
        self.assertIs(figure(built(), fig=target), target)
        self.assertEqual(len(target.axes), 3)

    def test_bicycle_raises(self):
        with self.assertRaises(ValueError):
            figure(load(FIXTURES / "bicycle.csv"))

    def test_show_draws_into_a_pyplot_figure(self):
        plt.close("all")
        pyplot_figure = plt.figure

        def as_in_matplotlib_3_10(*args, **kwargs):
            if args and isinstance(args[0], Figure) and args[0].canvas.manager is None:
                raise ValueError("The passed figure is not managed by pyplot")
            return pyplot_figure(*args, **kwargs)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "built.csv"
            run = built()
            support.write_run(path, run.data)
            with patch("matplotlib.pyplot.figure", side_effect=as_in_matplotlib_3_10), \
                    patch("matplotlib.pyplot.show") as show, contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(["energy", str(path), "--show"]), 0)
        self.assertEqual(len(plt.get_fignums()), 1)
        self.assertEqual(len(plt.gcf().axes), 3)
        show.assert_called_once_with()


class TestText(unittest.TestCase):
    def test_largest_residual_and_time(self):
        run = built()
        data = run.data.copy()
        data.loc[7, "energy.residual"] = -5.0
        out = energy.text(from_frame(data, kind="four_wheel", name="built"))
        self.assertIn("largest residual -5", out)
        self.assertIn(f"t = {data['time'][7]:.6g}", out)
        self.assertIn("largest residual", energy.text(run))

    def test_residual_printed_is_the_last_rows(self):
        # the residual ramps from 0.25 to -1 over the run
        data = built().data.copy()
        data["energy.residual"] = np.linspace(0.25, -1.0, len(data))
        lines = energy.text(from_frame(data, kind="four_wheel", name="built")).splitlines()
        self.assertIn("  residual\t-1", lines)
        self.assertNotIn("  residual\t0.25", lines)


if __name__ == "__main__":
    unittest.main()
