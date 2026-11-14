import contextlib
import io
import os
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.figure import Figure

from rallytools import cli
from rallytools.charts.handling import figure
from rallytools.handling import understeer
from rallytools.run import from_frame

ROWS = 1000
SPEED = 20.0


def frame(k, a_lat=None, speed=SPEED):
    if a_lat is None:
        a_lat = np.linspace(0.0, 6.0, ROWS)
    yaw_rate = a_lat / speed
    data = {
        "time": np.arange(len(a_lat)) * 0.001,
        "speed": np.full(len(a_lat), speed),
        "a_lat": a_lat,
        "yaw_rate": yaw_rate,
        "steer": 2.5 * yaw_rate / speed + k * a_lat,
    }
    for wheel in ("FL", "FR", "RL", "RR"):
        data[f"wheel.{wheel}.slipRatio"] = np.zeros(len(a_lat))
        data[f"wheel.{wheel}.slipAngle"] = 0.01 * a_lat * (1 if wheel[0] == "F" else 2)
    return pd.DataFrame(data)


def make(k, name="run", meta=None, **kwargs):
    meta = {"lf": 1.3, "lr": 1.2} if meta is None else meta
    return from_frame(frame(k, **kwargs), kind="four_wheel", name=name, meta=meta)


class TestUndersteer(unittest.TestCase):
    def test_gradient_of_a_built_run(self):
        result = understeer(make(0.002))
        self.assertAlmostEqual(result["gradient"], 0.002, delta=1e-12)
        self.assertAlmostEqual(result["gradient_deg"], 0.11459155902616465, delta=1e-12)

    def test_neutral_and_oversteer(self):
        self.assertAlmostEqual(understeer(make(0.0))["gradient"], 0.0, delta=1e-12)
        self.assertLess(understeer(make(-0.001))["gradient"], 0.0)

    def test_axle_slip_is_the_mean_of_its_wheels(self):
        result = understeer(make(0.002))
        a_lat = np.linspace(0.0, 6.0, ROWS)
        np.testing.assert_allclose(result["front_slip"], 0.01 * a_lat, atol=1e-15)
        np.testing.assert_allclose(result["rear_slip"], 0.02 * a_lat, atol=1e-15)

    def test_missing_lf_is_refused(self):
        with self.assertRaisesRegex(ValueError, "lf"):
            understeer(make(0.002, meta={"lr": 1.2}))

    def test_constant_a_lat_is_refused(self):
        with self.assertRaises(ValueError):
            understeer(make(0.002, a_lat=np.full(ROWS, 4.0)))

    def test_a_bicycle_run_is_refused(self):
        run = from_frame(frame(0.002)[["time", "speed", "a_lat", "yaw_rate", "steer"]].assign(yaw=0.0),
                         kind="bicycle", meta={"lf": 1.3, "lr": 1.2})
        with self.assertRaises(ValueError):
            understeer(run)

    def test_slow_rows_are_ignored(self):
        data = frame(0.002)
        slow = data.index[:100]
        data.loc[slow, "speed"] = 0.5
        data.loc[slow, "steer"] = 5.0
        run = from_frame(data, kind="four_wheel", meta={"lf": 1.3, "lr": 1.2})
        self.assertAlmostEqual(understeer(run)["gradient"], 0.002, delta=1e-12)


class TestHandlingChart(unittest.TestCase):
    def test_axes_and_lines(self):
        chart = figure([make(0.002, "a"), make(0.001, "b")])
        self.assertEqual(len(chart.axes), 2)
        self.assertEqual(len(chart.axes[0].lines), 4)
        self.assertEqual(len(chart.axes[1].lines), 4)

    def test_fitted_line_and_points(self):
        chart = figure([make(0.002)])
        points, fitted = chart.axes[0].lines
        x, y = fitted.get_data()
        slope = np.radians((y[-1] - y[0]) / (x[-1] - x[0]))
        self.assertAlmostEqual(slope, 0.002, delta=1e-12)
        self.assertEqual(fitted.get_linestyle(), "--")
        a_lat = np.linspace(0.0, 6.0, ROWS)
        expected = np.degrees(0.002 * a_lat)
        np.testing.assert_allclose(points.get_ydata(), expected, atol=1e-9)

    def test_figure_draws_into_a_given_figure(self):
        target = Figure()
        self.assertIs(figure([make(0.002)], fig=target), target)
        self.assertEqual(len(target.axes), 2)


class TestHandlingCommand(unittest.TestCase):
    def setUp(self):
        handle, self.path = tempfile.mkstemp(suffix=".csv")
        os.close(handle)
        with open(self.path, "w", encoding="utf-8") as f:
            f.write("# schema=6 lf=1.3 lr=1.2\n")
            frame(0.002).to_csv(f, index=False, lineterminator="\n")

    def tearDown(self):
        os.remove(self.path)

    def test_show_draws_into_a_pyplot_figure(self):
        plt.close("all")
        pyplot_figure = plt.figure

        def as_in_matplotlib_3_10(*args, **kwargs):
            # matplotlib 3.10 (the CI floor) refuses a figure pyplot did not make; 3.11 adopts it.
            if args and isinstance(args[0], Figure) and args[0].canvas.manager is None:
                raise ValueError("The passed figure is not managed by pyplot")
            return pyplot_figure(*args, **kwargs)

        with patch("matplotlib.pyplot.figure", side_effect=as_in_matplotlib_3_10), \
                patch("matplotlib.pyplot.show") as show, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(cli.main(["handling", self.path, "--show"]), 0)
        self.assertEqual(len(plt.get_fignums()), 1)
        self.assertEqual(len(plt.gcf().axes), 2)
        show.assert_called_once_with()
        plt.close("all")

    def test_prints_the_gradient(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            self.assertEqual(cli.main(["handling", self.path]), 0)
        self.assertIn("0.002", out.getvalue())
        self.assertIn("0.114592", out.getvalue())

    def test_window_picks_the_gradient_of_its_part_of_the_run(self):
        # the first half of the run has gradient 0.002, the second half 0.006
        time = np.arange(ROWS) * 0.001
        halves = frame(np.where(time < 0.5, 0.002, 0.006))
        with open(self.path, "w", encoding="utf-8") as f:
            f.write("# schema=6 lf=1.3 lr=1.2\n")
            halves.to_csv(f, index=False, lineterminator="\n")

        def gradient(*window):
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                self.assertEqual(cli.main(["handling", self.path, *window]), 0)
            return float(out.getvalue().split("understeer gradient ")[1].split()[0])

        self.assertAlmostEqual(gradient("--from", "0.5"), 0.006, delta=1e-9)
        self.assertAlmostEqual(gradient("--to", "0.4"), 0.002, delta=1e-9)
        self.assertAlmostEqual(gradient("--from", "0.1", "--to", "0.3"), 0.002, delta=1e-9)
        self.assertAlmostEqual(gradient("--from", "0.6", "--to", "0.9"), 0.006, delta=1e-9)
        # the whole run is neither half's gradient
        whole = gradient()
        self.assertGreater(abs(whole - 0.002), 1e-4)
        self.assertGreater(abs(whole - 0.006), 1e-4)

    def test_matplotlib_is_not_imported_without_save_or_show(self):
        code = (
            "import sys; from rallytools import cli; "
            f"assert cli.main(['handling', {self.path!r}]) == 0; "
            "assert 'matplotlib' not in sys.modules"
        )
        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
