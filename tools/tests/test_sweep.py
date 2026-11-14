import contextlib
import io
import math
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.figure import Figure

from rallytools import cli
from rallytools.charts.sweep import figure
from rallytools.run import from_frame, load
from rallytools.sweep import SweepResult, metric_value, sweep, values_of

from .support import need

EXE = os.environ.get("RALLYSIM_EXE")
VALUES = ["40kN/m", "50kN/m", "60kN/m"]


def _exe_ready(test):
    need(test, EXE and Path(EXE).exists(), "RALLYSIM_EXE is not set to a built rallysim")


class ValuesAndMetricsTest(unittest.TestCase):
    def test_values_of(self):
        self.assertEqual(values_of([], ("40", "60", "3"), "kN/m"), ["40kN/m", "50kN/m", "60kN/m"])
        self.assertEqual(values_of(["1", "2"], None, ""), ["1", "2"])
        with self.assertRaises(ValueError):
            values_of([], None, "")

    def test_metric_value(self):
        data = pd.DataFrame({"time": [0.0, 1.0, 2.0], "yaw_rate": [0.1, -0.5, 0.2], "speed": [1.0, 2.0, 3.0],
                             "yaw": [3.0, -3.0, -2.5]})
        run = from_frame(data, kind="bicycle", units={"yaw_rate": "rad/s", "speed": "m/s", "yaw": "rad"})
        self.assertEqual(metric_value(run, "maxabs:yaw_rate"), (28.64788975654116, "deg/s"))
        self.assertEqual(metric_value(run, "final:speed"), (3.0, "m/s"))
        # the heading counts on past pi: 3.0 -> -3.0 is a turn of 0.28 rad, not -6
        value, unit = metric_value(run, "final:yaw")
        self.assertAlmostEqual(value, math.degrees(-2.5 + 2 * math.pi), places=9)
        self.assertEqual(unit, "deg")
        with self.assertRaises(ValueError):
            metric_value(run, "nope:speed")
        with self.assertRaises(ValueError):
            metric_value(run, "max:nothing")


class SweepTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.result = None
        if EXE and Path(EXE).exists():
            cls.result = sweep(EXE, "fw_circle", "spring_front", VALUES, out=Path(cls.tmp.name))

    @classmethod
    def tearDownClass(cls):
        plt.close("all")
        cls.tmp.cleanup()

    def setUp(self):
        _exe_ready(self)

    def test_summary_matches_the_old_script(self):
        result = self.result
        self.assertEqual(result.values, VALUES)
        self.assertEqual(result.failed, [])
        summary = pd.read_csv(result.folder / "summary.csv")
        self.assertEqual(list(summary["value"]), VALUES)
        self.assertEqual(list(summary.columns),
                         ["value", "maxabs:beta", "maxabs:yaw_rate", "final:yaw", "final:speed"])
        for got, want in zip(summary["maxabs:beta"], (4.216120567, 4.231201378, 4.239165604)):
            self.assertAlmostEqual(got, want, delta=1e-6)
        for got, want in zip(summary["final:speed"], (6.005008555, 6.005051332, 6.005084736)):
            self.assertAlmostEqual(got, want, delta=1e-6)
        self.assertEqual(result.folder, Path(self.tmp.name) / "sweeps" / "fw_circle__spring_front")
        for path in result.paths:
            self.assertTrue(Path(path).exists())

    def test_a_refused_run_is_listed_not_fatal(self):
        with tempfile.TemporaryDirectory() as out:
            result = sweep(EXE, "fw_circle", "spring_front", ["40kN/m", "-1", "60kN/m"], out=Path(out))
        self.assertEqual(len(result.rows), 2)
        self.assertEqual([row["value"] for row in result.rows], ["40kN/m", "60kN/m"])
        self.assertEqual([value for value, _ in result.failed], ["-1"])
        self.assertTrue(result.failed[0][1])

    def test_every_run_failing_is_an_error(self):
        with tempfile.TemporaryDirectory() as out:
            with self.assertRaises(ValueError):
                sweep(EXE, "fw_circle", "spring_front", ["-1"], out=Path(out))

    def test_the_figure_draws_each_run_trace(self):
        chart = figure(self.result)
        self.assertIsInstance(chart, Figure)
        trace = chart.axes[-1]
        self.assertEqual(len(trace.lines), len(VALUES))
        for line, path in zip(trace.lines, self.result.paths):
            run = load(path)
            np.testing.assert_allclose(line.get_ydata(), np.degrees(run.column("yaw_rate")), rtol=0, atol=1e-12)
            np.testing.assert_allclose(line.get_xdata(), run.column("time"), rtol=0, atol=0)
        self.assertEqual(trace.get_ylabel(), "yaw_rate (deg/s)")
        top = chart.axes[0]
        np.testing.assert_allclose(top.lines[0].get_xdata(), [40, 50, 60])
        np.testing.assert_allclose(top.lines[0].get_ydata(), [row["maxabs:beta"] for row in self.result.rows])

    def test_the_trace_can_be_another_column(self):
        chart = figure(self.result, trace="speed")
        trace = chart.axes[-1]
        for line, path in zip(trace.lines, self.result.paths):
            np.testing.assert_allclose(line.get_ydata(), load(path).column("speed"), rtol=0, atol=0)
        with self.assertRaises(ValueError):
            figure(self.result, trace="no_such_column")

    def test_figure_draws_into_a_given_figure(self):
        target = Figure()
        self.assertIs(figure(self.result, fig=target), target)
        self.assertEqual(len(target.axes), 5)

    def test_relative_save_path_through_cli(self):
        with tempfile.TemporaryDirectory() as work:
            previous = os.getcwd()
            os.chdir(work)
            try:
                with patch("rallytools.cli.OUT", Path(work) / "out"):
                    code = cli.main(["sweep", "fw_circle", "spring_front", "40kN/m", "50kN/m", "60kN/m",
                                     "--exe", EXE, "--save", "picture.png"])
                self.assertEqual(code, 0)
                self.assertTrue((Path(work) / "picture.png").stat().st_size > 0)
            finally:
                os.chdir(previous)

    def test_held_settings_reach_every_run(self):
        # read back from line 1 of each run: the default mass is 1200 and mu_front 1
        with tempfile.TemporaryDirectory() as work:
            with patch("rallytools.cli.OUT", Path(work) / "out"), contextlib.redirect_stdout(io.StringIO()):
                code = cli.main(["sweep", "fw_circle", "spring_front", "40kN/m", "50kN/m", "60kN/m",
                                 "--set", "mass=1234", "--set", "mu_front=0.8", "--exe", EXE,
                                 "--save", str(Path(work) / "held.png")])
            self.assertEqual(code, 0)
            files = sorted((Path(work) / "out" / "sweeps" / "fw_circle__spring_front").glob("0*.csv"))
            self.assertEqual(len(files), 3)
            for path, spring in zip(files, (40000.0, 50000.0, 60000.0)):
                meta = load(path).meta
                self.assertEqual(meta["spring_front"], spring, path.name)
                self.assertEqual(meta["mass"], 1234.0, path.name)
                self.assertEqual(meta["mu_front"], 0.8, path.name)

    def test_show_draws_into_a_pyplot_figure(self):
        plt.close("all")
        pyplot_figure = plt.figure

        def as_in_matplotlib_3_10(*args, **kwargs):
            # matplotlib 3.10 (the CI floor) refuses a figure pyplot did not make; 3.11 adopts it.
            if args and isinstance(args[0], Figure) and args[0].canvas.manager is None:
                raise ValueError("The passed figure is not managed by pyplot")
            return pyplot_figure(*args, **kwargs)

        with tempfile.TemporaryDirectory() as work:
            with patch("matplotlib.pyplot.figure", side_effect=as_in_matplotlib_3_10), \
                    patch("matplotlib.pyplot.show") as show, \
                    patch("rallytools.cli.OUT", Path(work) / "out"):
                code = cli.main(["sweep", "fw_circle", "spring_front", "40kN/m", "50kN/m", "--exe", EXE,
                                 "--save", str(Path(work) / "shown.png"), "--show"])
        self.assertEqual(code, 0)
        self.assertEqual(len(plt.get_fignums()), 1)
        self.assertEqual(len(plt.gcf().axes[-1].lines), 2)
        show.assert_called_once_with()
        plt.close("all")


def _synthetic(count, metrics=("maxabs:yaw_rate",)):
    """A sweep result made in memory (no rallysim): one yaw_rate trace per value."""
    t = np.linspace(0.0, 2.0, 21)
    values = [f"{40 + 5 * i}kN/m" for i in range(count)]
    runs = [from_frame(pd.DataFrame({"time": t, "yaw_rate": 0.1 * (i + 1) * np.sin(t)}), kind="bicycle",
                       units={"yaw_rate": "rad/s"}) for i in range(count)]
    rows = [{"value": value, **{spec: 1.0 + i for spec in metrics}} for i, value in enumerate(values)]
    return SweepResult("fw_circle", "spring_front", (), tuple(metrics), Path("."), values, [], runs, rows, [])


class FigureWidthTest(unittest.TestCase):
    def tearDown(self):
        plt.close("all")

    def test_a_sweep_of_one_or_two_metrics_is_at_least_ten_inches_wide(self):
        self.assertGreaterEqual(figure(_synthetic(6)).get_size_inches()[0], 10)
        two = figure(_synthetic(6, ("maxabs:yaw_rate", "final:yaw_rate")))
        self.assertGreaterEqual(two.get_size_inches()[0], 10)
        # more metrics keep 4.2 in each
        five = ("max:yaw_rate", "min:yaw_rate", "maxabs:yaw_rate", "mean:yaw_rate", "final:yaw_rate")
        self.assertAlmostEqual(figure(_synthetic(3, five)).get_size_inches()[0], 21.0)

    def test_x_label_carries_the_unit_the_values_share(self):
        for chart in (figure(_synthetic(3)), figure(_synthetic(3), fig=Figure())):
            self.assertEqual(chart.axes[0].get_xlabel(), "spring_front (kN/m)")
        # values with different units, or none, leave the label bare
        mixed = _synthetic(2)
        mixed.values[1] = "50N/m"
        self.assertEqual(figure(mixed).axes[0].get_xlabel(), "spring_front")
        plain = _synthetic(2)
        plain.values[:] = ["40", "50"]
        self.assertEqual(figure(plain).axes[0].get_xlabel(), "spring_front")

    def test_the_width_holds_in_a_given_figure(self):
        target = Figure()
        figure(_synthetic(3), fig=target)
        self.assertGreaterEqual(target.get_size_inches()[0], 10)

    def test_the_trace_legend_lies_inside_the_figure(self):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        for count in (2, 6, 12):
            chart = figure(_synthetic(count))
            FigureCanvasAgg(chart).draw()
            legend = chart.axes[-1].get_legend()
            box, page = legend.get_window_extent(), chart.bbox
            self.assertGreaterEqual(box.x0, page.x0, count)
            self.assertLessEqual(box.x1, page.x1, count)
            self.assertGreaterEqual(box.y0, page.y0, count)
            self.assertLessEqual(box.y1, page.y1, count)

    def test_the_legend_has_no_more_columns_than_values_or_than_fit(self):
        two = figure(_synthetic(2)).axes[-1].get_legend()
        self.assertEqual(two._ncols, 2)
        many = figure(_synthetic(12)).axes[-1].get_legend()
        self.assertLessEqual(many._ncols, 5)  # about 1.6 in a column in a 10 in figure


if __name__ == "__main__":
    unittest.main()
