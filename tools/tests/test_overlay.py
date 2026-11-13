from pathlib import Path
import unittest
from unittest.mock import patch

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import numpy as np
import pandas as pd

from rallytools import cli
from rallytools.charts.overlay import figure, table
from rallytools.run import from_frame, load

FIXTURES = Path(__file__).parent / "fixtures"


def _built(offset):
    t = np.round(np.arange(0.0, 1.0001, 0.05), 6)
    speed = 10.0 + t
    frame = pd.DataFrame({"time": t, "speed": speed, "yaw": 0.0 * t, "pos.x": t, "pos.y": 0.0 * t})
    shifted = frame.copy()
    shifted["speed"] = speed + np.where(t >= 0.2, offset, 0.0)
    return (from_frame(frame, kind="bicycle", name="a"), from_frame(shifted, kind="bicycle", name="b"))


class TestOverlay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.springs = load(FIXTURES / "four_wheel_springs.csv")
        cls.unsprung = load(FIXTURES / "four_wheel_unsprung.csv")
        cls.schema5 = load(FIXTURES / "four_wheel_schema5.csv")

    def test_figure_of_two_has_path_and_five_channels(self):
        chart = figure([self.springs, self.unsprung])
        self.assertEqual([len(axes.lines) for axes in chart.axes], [4] + [2] * 10)

    def test_figure_with_two_channels(self):
        chart = figure([self.springs, self.unsprung], ["speed", "pitch"])
        self.assertEqual([len(axes.lines) for axes in chart.axes], [4, 2, 2, 2, 2])

    def test_figure_draws_into_a_given_figure(self):
        target = Figure()
        self.assertIs(figure([self.springs, self.unsprung], fig=target), target)
        self.assertEqual(len(target.axes), 11)

    def test_table_of_two(self):
        text = table([self.springs, self.unsprung])
        self.assertIn("line-1 facts that differ", text)
        lines = text.splitlines()
        start = next(i for i, line in enumerate(lines) if line.startswith("channel"))
        names = [line.split()[0] for line in lines[start + 1:]]
        self.assertEqual(names, ["speed", "yaw_rate", "beta", "a_lat", "pitch"])
        self.assertEqual(float(lines[start + 1].split()[1]), 0.0)

    def test_three_runs(self):
        runs = [self.springs, self.unsprung, self.schema5]
        chart = figure(runs)
        self.assertEqual(len(chart.axes), 11)
        self.assertEqual(len(chart.axes[0].lines), 6)
        self.assertEqual(table(runs).count("channel "), 2)

    def test_difference_is_later_minus_first_at_the_right_time(self):
        a, b = _built(0.5)
        speed = next(line for line in table([a, b], ["speed"]).splitlines() if line.startswith("speed"))
        fields = speed.split()
        self.assertAlmostEqual(float(fields[1]), 0.5)
        self.assertAlmostEqual(float(fields[2]), 0.2)

    def test_unknown_channel_and_mixed_kinds_are_refused(self):
        with self.assertRaises(ValueError):
            figure([self.springs, self.unsprung], ["nope"])
        with self.assertRaises(ValueError):
            table([self.springs, self.unsprung], ["nope"])
        bicycle = load(FIXTURES / "bicycle.csv")
        with self.assertRaises(ValueError):
            figure([bicycle, self.springs])
        with self.assertRaises(ValueError):
            table([bicycle, self.springs])

    def test_show_draws_into_a_pyplot_figure(self):
        plt.close("all")
        pyplot_figure = plt.figure

        def as_in_matplotlib_3_10(*args, **kwargs):
            # matplotlib 3.10 (the CI floor) refuses a figure pyplot did not make; 3.11 adopts it.
            if args and isinstance(args[0], Figure) and args[0].canvas.manager is None:
                raise ValueError("The passed figure is not managed by pyplot")
            return pyplot_figure(*args, **kwargs)

        with patch("matplotlib.pyplot.figure", side_effect=as_in_matplotlib_3_10), \
                patch("matplotlib.pyplot.show") as show:
            self.assertEqual(
                cli.main(["overlay", str(FIXTURES / "four_wheel_springs.csv"),
                          str(FIXTURES / "four_wheel_unsprung.csv")]),
                0,
            )
        self.assertEqual(len(plt.get_fignums()), 1)
        self.assertEqual(len(plt.gcf().axes), 11)
        show.assert_called_once_with()


def _run(name, t, **columns):
    t = np.asarray(t, dtype=float)
    frame = pd.DataFrame({"time": t, "pos.x": t, "pos.y": 0.0 * t})
    for key, value in columns.items():
        frame[key] = value * np.ones(len(t)) if np.isscalar(value) else value
    return from_frame(frame, kind="bicycle", name=name)


def _lines(axes):
    return axes.lines[:-1]  # the last line of a difference axes is the zero line


class TestOverlayData(unittest.TestCase):
    def test_difference_sign_and_step(self):
        t = np.round(np.arange(0.0, 1.0001, 0.05), 6)
        a = _run("a", t, speed=10.0)
        b = _run("b", t, speed=10.0 + np.where(t >= 0.2, 0.5, 0.0))
        diff = _lines(figure([a, b], ["speed"]).axes[2])[0]
        np.testing.assert_allclose(diff.get_ydata(), np.where(t >= 0.2, 0.5, 0.0), atol=1e-12)

    def test_each_later_run_is_differenced_against_the_first(self):
        t = np.round(np.arange(0.0, 1.0001, 0.05), 6)
        runs = [_run(name, t, speed=10.0 + offset) for name, offset in (("a", 0.0), ("b", 0.5), ("c", 1.0))]
        lines = _lines(figure(runs, ["speed"]).axes[2])
        self.assertEqual(len(lines), 2)
        np.testing.assert_allclose(lines[0].get_ydata(), 0.5, atol=1e-12)
        np.testing.assert_allclose(lines[1].get_ydata(), 1.0, atol=1e-12)

    def test_difference_is_on_the_first_runs_times(self):
        ta = np.round(np.arange(0.0, 1.0001, 0.002), 6)
        tb = np.round(np.arange(0.0, 1.0001, 0.001), 6)
        a, b = _run("a", ta, speed=10.0), _run("b", tb, speed=10.0)
        diff = _lines(figure([a, b], ["speed"]).axes[2])[0]
        np.testing.assert_array_equal(diff.get_xdata(), ta)

    def test_angle_difference_is_wrapped(self):
        t = np.round(np.arange(0.0, 1.0001, 0.1), 6)
        a, b = _run("a", t, yaw=3.1), _run("b", t, yaw=-3.1)
        diff = _lines(figure([a, b], ["yaw"]).axes[2])[0]
        np.testing.assert_allclose(diff.get_ydata(), 2 * np.pi - 6.2, atol=1e-12)

    def test_beta_is_left_out_at_a_standstill(self):
        t = np.round(np.arange(0.0, 2.0, 0.01), 6)
        speed = np.where(np.arange(len(t)) < 100, 0.1, 5.0)
        a = _run("a", t, speed=speed, beta=0.1)
        b = _run("b", t, speed=speed, beta=0.2)
        y = _lines(figure([a, b], ["beta"]).axes[2])[0].get_ydata()
        self.assertTrue(np.isnan(y[:100]).all())
        self.assertTrue(np.isfinite(y[100:]).all())

    def test_first_run_is_dashed_and_the_others_solid(self):
        t = np.round(np.arange(0.0, 1.0001, 0.05), 6)
        runs = [_run(name, t, speed=10.0 + i) for i, name in enumerate("abc")]
        chart = figure(runs, ["speed"])
        for axes in (chart.axes[0], chart.axes[1]):
            lines = [line for line in axes.lines if len(line.get_xdata()) > 1]
            self.assertEqual([line.get_linestyle() for line in lines], ["--", "-", "-"])


if __name__ == "__main__":
    unittest.main()
