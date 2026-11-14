import contextlib
import io
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import numpy as np
import pandas as pd

from rallytools import cli
from rallytools.charts.characteristics import figure
from rallytools.run import WHEELS, from_frame, load

from . import support

FIXTURES = Path(__file__).parent / "fixtures"
EXE = os.environ.get("RALLYSIM_EXE")

N = 41
D = np.linspace(-0.05, 0.15, N)
V = np.linspace(-1.0, 1.0, N)


def spring(index):
    return 50000.0 * np.maximum(D, 0.0) + 1000.0 * index


def damper(index):
    return 3000.0 * V + 10.0 * index


def stop(index):
    return np.where(D > 0.1, 1e6 * (D - 0.1) ** 2, 0.0) + 5.0 * index


def built(meta=None):
    columns = {"time": np.arange(N) * 0.01}
    for index, wheel in enumerate(WHEELS):
        columns[f"wheel.{wheel}.slipRatio"] = np.zeros(N)
        columns[f"wheel.{wheel}.deflection"] = D + 0.001 * index
        columns[f"wheel.{wheel}.deflVel"] = V + 0.01 * index
        columns[f"wheel.{wheel}.springF"] = spring(index)
        columns[f"wheel.{wheel}.damperF"] = damper(index)
        columns[f"wheel.{wheel}.stopF"] = stop(index)
    return from_frame(pd.DataFrame(columns), kind="four_wheel", name="known", schema=6, meta=meta)


def verticals(axis):
    return sorted(float(line.get_xdata()[0]) for line in axis.lines)


class TestCharacteristics(unittest.TestCase):
    def tearDown(self):
        plt.close("all")

    def test_each_axes_holds_each_wheels_own_pairs(self):
        chart = figure(built())
        self.assertIsInstance(chart, Figure)
        self.assertEqual(len(chart.axes), 3)
        for axis, (x, y) in zip(chart.axes, (("deflection", "springF"), ("deflVel", "damperF"),
                                             ("deflection", "stopF"))):
            self.assertEqual(len(axis.collections), 4)
            for index, (wheel, points) in enumerate(zip(WHEELS, axis.collections)):
                xs = (D + 0.001 * index) if x == "deflection" else (V + 0.01 * index)
                expected = np.column_stack([xs, y_values(y, index)])
                self.assertTrue(np.array_equal(points.get_offsets(), expected), f"{y} {wheel}")
                self.assertEqual(points.get_label(), wheel)

    def test_rest_deflection_lines(self):
        chart = figure(built({"rest_deflection_front": 0.05, "rest_deflection_rear": 0.06}))
        self.assertEqual(verticals(chart.axes[0]), [0.05, 0.06])
        self.assertEqual(verticals(chart.axes[1]), [])
        self.assertEqual(verticals(chart.axes[2]), [0.05, 0.06])
        chart = figure(built())
        for axis in chart.axes:
            self.assertEqual(verticals(axis), [])

    def test_old_file_and_wrong_kind_raise(self):
        with self.assertRaisesRegex(ValueError, "stopF"):
            figure(load(FIXTURES / "four_wheel_schema5.csv"))
        with self.assertRaises(ValueError):
            figure(load(FIXTURES / "bicycle.csv"))

    def test_draws_into_a_given_figure(self):
        target = Figure()
        self.assertIs(figure(built(), fig=target), target)
        self.assertEqual(len(target.axes), 3)

    def test_show_draws_into_a_pyplot_figure(self):
        plt.close("all")
        pyplot_figure = plt.figure

        def as_in_matplotlib_3_10(*args, **kwargs):
            if args and isinstance(args[0], Figure) and args[0].canvas.manager is None:
                raise ValueError("The passed figure is not managed by pyplot")
            return pyplot_figure(*args, **kwargs)

        with patch("matplotlib.pyplot.figure", side_effect=as_in_matplotlib_3_10), \
                patch("matplotlib.pyplot.show") as show:
            self.assertEqual(
                cli.main(["characteristics", str(FIXTURES / "four_wheel_unsprung.csv"), "--show"]), 0)
        self.assertEqual(len(plt.get_fignums()), 1)
        self.assertEqual(len(plt.gcf().axes), 3)
        show.assert_called_once_with()

    def test_save_writes_a_png(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "sub" / "ch.png"
            with contextlib.redirect_stdout(io.StringIO()):
                code = cli.main(["characteristics", str(FIXTURES / "four_wheel_unsprung.csv"),
                                 "--save", str(output)])
            self.assertEqual(code, 0)
            self.assertGreater(output.stat().st_size, 0)

    def test_bump_stop_starts_where_its_travel_says(self):
        support.need(self, EXE and os.path.isfile(EXE), "RALLYSIM_EXE must name an existing rallysim executable")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "fw_showcase.csv"
            result = subprocess.run(
                [EXE, "fw_showcase", str(output), "--set", "vertical=unsprung",
                 "--set", "bump_travel_front=0.03", "--set", "bump_travel_rear=0.03"],
                capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            run = load(output)
        chart = figure(run)
        found = False
        for wheel, points in zip(WHEELS, chart.axes[2].collections):
            axle = "front" if wheel in ("FL", "FR") else "rear"
            start = run.meta[f"rest_deflection_{axle}"] + 0.03
            offsets = points.get_offsets()
            engaged = offsets[offsets[:, 1] > 0]
            if len(engaged):
                found = True
                self.assertTrue(np.all(engaged[:, 0] > start), wheel)
        self.assertTrue(found, "the bump stop never engaged")


def y_values(name, index):
    return {"springF": spring, "damperF": damper, "stopF": stop}[name](index)


if __name__ == "__main__":
    unittest.main()
