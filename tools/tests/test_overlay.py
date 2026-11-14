from pathlib import Path
import unittest
from unittest.mock import patch

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import numpy as np
import pandas as pd

from rallytools import cli
from rallytools.charts.overlay import _prepare, figure, table
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

    def test_the_second_table_block_is_the_third_run_against_the_first(self):
        t = np.round(np.arange(0.0, 1.0001, 0.05), 6)
        runs = [_run(name, t, speed=10.0 + offset) for name, offset in (("a", 0.0), ("b", 0.5), ("c", 1.0))]
        first, second = table(runs, ["speed"]).split("\n\n")
        self.assertEqual(first.splitlines()[:2], ["A = a", "B = b"])
        self.assertEqual(second.splitlines()[:2], ["A = a", "C = c"])

        def speed_row(block):
            return next(line for line in block.splitlines() if line.startswith("speed")).split()

        # largest |difference|, then both final values
        row = speed_row(first)
        self.assertEqual([float(v) for v in (row[1], row[3], row[4])], [0.5, 10.0, 10.5])
        row = speed_row(second)
        self.assertEqual([float(v) for v in (row[1], row[3], row[4])], [1.0, 10.0, 11.0])

    def test_first_run_is_dashed_and_the_others_solid(self):
        t = np.round(np.arange(0.0, 1.0001, 0.05), 6)
        runs = [_run(name, t, speed=10.0 + i) for i, name in enumerate("abc")]
        chart = figure(runs, ["speed"])
        for axes in (chart.axes[0], chart.axes[1]):
            lines = [line for line in axes.lines if len(line.get_xdata()) > 1]
            self.assertEqual([line.get_linestyle() for line in lines], ["--", "-", "-"])


def _with_line1(name, meta, t=None, **columns):
    t = np.round(np.arange(0.0, 1.0001, 0.05), 6) if t is None else t
    frame = pd.DataFrame({"time": t, "pos.x": t, "pos.y": 0.0 * t})
    for key, value in columns.items():
        frame[key] = value * np.ones(len(t)) if np.isscalar(value) else value
    return from_frame(frame, kind="bicycle", name=name, meta=meta)


class TestLabels(unittest.TestCase):
    def pair(self):
        return (_with_line1("x", {"spring_front": 50000.0, "curve_front": 2.0}),
                _with_line1("y", {"spring_front": 60000.0, "curve_front": 1.0}))

    def test_label_names_what_differs_with_curves_by_name(self):
        a, b = self.pair()
        lines = table([a, b], ["pos.x"]).splitlines()
        self.assertEqual(lines[0], "A = x: spring_front (50000), curve_front (tmeasy)")
        self.assertEqual(lines[1], "B = y: spring_front (60000), curve_front (peak_slide)")
        self.assertIn("line-1 facts that differ: curve_front tmeasy -> peak_slide, spring_front 50000 -> 60000", lines)
        legend = [text.get_text() for text in figure([a, b], ["pos.x"]).axes[0].get_legend().get_texts()]
        self.assertEqual(legend, ["x: spring_front (50000), curve_front (tmeasy)",
                                  "y: spring_front (60000), curve_front (peak_slide)"])

    def test_no_more_than_two_then_the_count(self):
        a, b = self.pair()
        a.meta["mass"], b.meta["mass"] = 1000.0, 1100.5
        a.meta["lf"], b.meta["lf"] = 1.1, 1.2
        lines = table([a, b], ["pos.x"]).splitlines()
        self.assertEqual(lines[0], "A = x: spring_front (50000), curve_front (tmeasy) +2 more")
        self.assertEqual(lines[1], "B = y: spring_front (60000), curve_front (peak_slide) +2 more")
        del a.meta["lf"], b.meta["lf"]
        self.assertEqual(table([a, b], ["pos.x"]).splitlines()[1],
                         "B = y: spring_front (60000), curve_front (peak_slide) +1 more")

    def test_identical_line_1_keeps_the_origin(self):
        a = _with_line1("x", {"spring_front": 50000.0, "curve_front": 2.0})
        b = _with_line1("y", {"spring_front": 50000.0, "curve_front": 2.0})
        lines = table([a, b], ["pos.x"]).splitlines()
        self.assertEqual(lines[:3], ["A = x", "B = y", "line-1 facts: the same"])
        # only line 1 counts: a run with another column or another speed is not a differing parameter
        c = _with_line1("z", {"spring_front": 50000.0, "curve_front": 2.0}, speed=3.0)
        self.assertEqual(table([a, c], ["pos.x"]).splitlines()[1], "B = z")

    def test_later_runs_are_compared_with_the_first(self):
        a = _with_line1("x", {"spring_front": 50000.0})
        b = _with_line1("y", {"spring_front": 60000.0})
        c = _with_line1("z", {"spring_front": 50000.0})
        _, _, labels = _prepare([a, b, c], ["pos.x"], None)
        self.assertEqual(labels, ["x: spring_front (50000)", "y: spring_front (60000)", "z"])

    def test_names_that_repeat_become_letters_first(self):
        a = _with_line1("run", {"mass": 1000.0})
        b = _with_line1("run", {"mass": 1200.0})
        _, _, labels = _prepare([a, b], ["pos.x"], None)
        self.assertEqual(labels, ["A: mass (1000)", "B: mass (1200)"])

    def test_given_labels_are_used_as_they_are(self):
        a, b = self.pair()
        lines = table([a, b], ["pos.x"], ["old", "new"]).splitlines()
        self.assertEqual(lines[:2], ["A = old", "B = new"])
        legend = [text.get_text() for text in figure([a, b], ["pos.x"], ["old", "new"]).axes[0].get_legend().get_texts()]
        self.assertEqual(legend, ["old", "new"])
        # origins that are to be described (what compare.py passes) are described only on request
        text = table([a, b], ["pos.x"], ["snap", "current"], describe=True).splitlines()
        self.assertEqual(text[1], "B = current: spring_front (60000), curve_front (peak_slide)")
        self.assertEqual(table([a, b], ["pos.x"], ["snap", "current"]).splitlines()[:2],
                         ["A = snap", "B = current"])

    def test_curve_by_name_for_every_curve_key(self):
        a = _with_line1("x", {"curve": 0.0, "curve_rear": 1.0})
        b = _with_line1("y", {"curve": 2.0, "curve_rear": 2.0})
        self.assertEqual(table([a, b], ["pos.x"]).splitlines()[0], "A = x: curve (linear), curve_rear (peak_slide)")


class TestInputsNote(unittest.TestCase):
    def runs(self, **second):
        t = np.round(np.arange(0.0, 1.0001, 0.05), 6)
        first = {"steer": 0.0, "throttle": 0.5, "brake": 0.0, "handbrake": 0.0}
        other = dict(first)
        for key, value in second.items():
            other[key] = np.where(t >= value[0], value[1], first[key])
        return _with_line1("a", {}, speed=10.0, **first), _with_line1("b", {}, speed=10.0, **other)

    def lines(self, a, b):
        return table([a, b], ["speed"]).splitlines()

    def test_the_earliest_differing_column_and_time(self):
        a, b = self.runs(steer=(0.3, 0.1))
        lines = self.lines(a, b)
        self.assertIn("inputs differ (closed loop): steer from 0.3 s", lines)
        # after the line-1 facts, before the channel table
        self.assertLess(lines.index("line-1 facts: the same"), lines.index("inputs differ (closed loop): steer from 0.3 s"))
        a, b = self.runs(steer=(0.3, 0.1), brake=(0.2, 0.4))
        self.assertIn("inputs differ (closed loop): brake from 0.2 s", self.lines(a, b))
        self.assertEqual(sum(line.startswith("inputs differ") for line in self.lines(a, b)), 1)

    def test_the_second_run_is_the_one_read_as_b(self):
        a, b = self.runs(steer=(0.3, 0.1))
        self.assertTrue(any(line.startswith("inputs differ") for line in self.lines(a, b)))
        # a run compared with itself agrees; the first run's column is not read twice
        self.assertFalse(any(line.startswith("inputs differ") for line in self.lines(a, a)))
        self.assertFalse(any(line.startswith("inputs differ") for line in self.lines(b, b)))

    def test_equal_inputs_and_a_tiny_difference_give_no_line(self):
        a, b = self.runs()
        self.assertFalse(any(line.startswith("inputs differ") for line in self.lines(a, b)))
        b.data["steer"] = b.data["steer"] + 1e-12
        self.assertFalse(any(line.startswith("inputs differ") for line in self.lines(a, b)))

    def test_runs_without_input_columns_are_fine(self):
        a, b = _built(0.5)
        self.assertFalse(any(line.startswith("inputs differ") for line in table([a, b], ["speed"]).splitlines()))
        # one run with them and one without is the same
        c, d = self.runs(steer=(0.3, 0.1))
        self.assertFalse(any(line.startswith("inputs differ") for line in table([c, b], ["speed"]).splitlines()))


if __name__ == "__main__":
    unittest.main()
