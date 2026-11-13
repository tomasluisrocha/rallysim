import os
import contextlib
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import matplotlib.pyplot as plt
from matplotlib.figure import Figure

from rallytools import cli
from rallytools.run import load
from rallytools.charts.suspension import figure
from rallytools.run import from_frame

import numpy as np
import pandas as pd


FIXTURES = Path(__file__).parent / "fixtures"


class TestSuspensionChart(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runs = [
            load(FIXTURES / "four_wheel_springs.csv", name="springs"),
            load(FIXTURES / "four_wheel_unsprung.csv", name="unsprung"),
        ]

    def tearDown(self):
        plt.close("all")

    def test_structure_and_titles(self):
        chart = figure(self.runs)
        titles = [axis.get_title() for axis in chart.axes]
        self.assertEqual(len(chart.axes), 12)
        self.assertEqual(
            titles,
            [
                "roll (deg) over time",
                "roll (deg) against a_lat (m/s^2)",
                "lateral load transfer (N): front solid, rear dashed",
                "front share of the transfer",
                "Fz FL (N)", "Fz FR (N)", "Fz RL (N)", "Fz RR (N)",
                "strut deflection FL (m)", "strut deflection FR (m)",
                "strut deflection RL (m)", "strut deflection RR (m)",
            ],
        )

    def test_lines_and_single_legend(self):
        chart = figure(self.runs)
        self.assertEqual([len(axis.lines) for axis in chart.axes], [2, 2, 4, 2] + [2] * 8)
        self.assertEqual(
            [text.get_text() for text in chart.axes[0].get_legend().get_texts()],
            ["springs", "unsprung"],
        )
        self.assertEqual(sum(axis.get_legend() is not None for axis in chart.axes), 1)

    def test_colours_are_consistent_per_run(self):
        chart = figure(self.runs)
        for run_index in range(2):
            colours = []
            for axis in chart.axes:
                line_index = run_index * 2 if axis is chart.axes[2] else run_index
                colours.append(axis.lines[line_index].get_color())
            self.assertEqual(len(set(colours)), 1)
        self.assertNotEqual(chart.axes[0].lines[0].get_color(), chart.axes[0].lines[1].get_color())

    def test_figure_does_not_use_pyplot(self):
        plt.close("all")
        figure(self.runs)
        self.assertEqual(plt.get_fignums(), [])

    def test_schema_five_and_single_run(self):
        self.assertEqual(len(figure([load(FIXTURES / "four_wheel_schema5.csv")]).axes), 12)
        self.assertEqual(len(figure(self.runs[:1]).axes), 12)

    def test_save(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "suspension.png"
            self.assertEqual(
                cli.main(["suspension", str(FIXTURES / "four_wheel_springs.csv"),
                          str(FIXTURES / "four_wheel_unsprung.csv"), "--save", str(path)]),
                0,
            )
            self.assertGreater(path.stat().st_size, 10_000)
            self.assertEqual(path.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")

    def test_cli_without_chart_does_not_import_matplotlib(self):
        fixture = str(FIXTURES / "four_wheel_springs.csv")
        code = (
            "import sys; from rallytools import cli; "
            f"assert cli.main(['suspension', {fixture!r}]) == 0; "
            "assert 'matplotlib' not in sys.modules"
        )
        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

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
                cli.main(["suspension", str(FIXTURES / "four_wheel_springs.csv"), "--show"]),
                0,
            )
        self.assertEqual(len(plt.get_fignums()), 1)
        self.assertEqual(len(plt.gcf().axes), 12)
        show.assert_called_once_with()

    def test_figure_draws_into_a_given_figure(self):
        target = Figure()
        self.assertIs(figure(self.runs, fig=target), target)
        self.assertEqual(len(target.axes), 12)

    def test_save_and_show_are_rejected_together(self):
        error = io.StringIO()
        with contextlib.redirect_stderr(error):
            result = cli.main([
                "suspension", str(FIXTURES / "four_wheel_springs.csv"),
                "--save", "chart.png", "--show",
            ])
        self.assertEqual(result, 2)
        self.assertIn("mutually exclusive", error.getvalue())


def marked_run(name="m", lock=(0.2, 0.4), spin=(0.5, 0.7), rows=1000, dt=0.001):
    t = np.arange(rows) * dt
    data = {"time": t, "pos.z": np.full(rows, 0.5), "speed": np.full(rows, 20.0),
            "roll": np.zeros(rows), "a_lat": np.zeros(rows)}
    for w in ("FL", "FR", "RL", "RR"):
        data[f"wheel.{w}.Fz"] = np.full(rows, 3000.0)
        data[f"wheel.{w}.deflection"] = np.full(rows, 0.05)
        data[f"wheel.{w}.slipRatio"] = np.zeros(rows)
        data[f"wheel.{w}.omega"] = np.full(rows, 66.7)
        data[f"wheel.{w}.stopF"] = np.zeros(rows)
    data["wheel.FL.slipRatio"][(t >= lock[0]) & (t < lock[1])] = -1.0
    data["wheel.RR.slipRatio"][(t >= spin[0]) & (t < spin[1])] = 0.5
    return from_frame(pd.DataFrame(data), kind="four_wheel", meta={"wheel_radius": 0.3}, name=name)


class TestSuspensionMarks(unittest.TestCase):
    def tearDown(self):
        plt.close("all")

    def check_marks(self, chart, plain):
        self.assertEqual(len(chart.axes), 12)
        for i, (axis, base) in enumerate(zip(chart.axes, plain.axes)):
            extra = len(axis.lines) - len(base.lines)
            if i == 1:
                self.assertEqual(extra, 0)
                continue
            self.assertEqual(extra, 2, axis.get_title())
            new = axis.lines[len(base.lines):]
            self.assertEqual(sorted(float(line.get_xdata()[0]) for line in new), [0.2, 0.5])
            for line in new:
                self.assertEqual(list(line.get_xdata()), [line.get_xdata()[0]] * 2)
                self.assertEqual(list(line.get_ydata()), [0, 1])

    def test_marks_at_event_starts_in_every_time_axis(self):
        run = marked_run()
        chart = figure([run], marks=True)
        self.check_marks(chart, figure([run]))
        self.assertEqual([t.get_text() for t in chart.axes[0].get_legend().get_texts()],
                         ["m", "lock", "spin"])

    def test_marks_are_of_the_first_run_only(self):
        first, second = marked_run("a"), marked_run("b", lock=(0.6, 0.8), spin=(0.1, 0.3))
        chart = figure([first, second], marks=True)
        self.check_marks(chart, figure([first, second]))

    def test_no_events_no_marks(self):
        runs = [load(FIXTURES / "four_wheel_springs.csv"), load(FIXTURES / "four_wheel_unsprung.csv")]
        plain, marked = figure(runs), figure(runs, marks=True)
        self.assertEqual([len(a.lines) for a in marked.axes], [len(a.lines) for a in plain.axes])

    def test_lift_and_touchdown(self):
        # all four tyres off the ground from 0.1 to 0.3 s: four lift events at 0.1, one landing at 0.3
        run = marked_run(lock=(5, 5), spin=(5, 5))
        airborne = (run.data["time"] >= 0.1) & (run.data["time"] < 0.3)
        for w in ("FL", "FR", "RL", "RR"):
            run.data.loc[airborne, f"wheel.{w}.Fz"] = 0.0
        chart, plain = figure([run], marks=True), figure([run])
        for i, (axis, base) in enumerate(zip(chart.axes, plain.axes)):
            new = sorted(float(line.get_xdata()[0]) for line in axis.lines[len(base.lines):])
            self.assertEqual(new, [] if i == 1 else [0.1] * 4 + [0.3], axis.get_title())
        self.assertEqual([t.get_text() for t in chart.axes[0].get_legend().get_texts()], ["m", "lift", "touchdown"])

    def test_no_wheel_radius_no_marks(self):
        run = marked_run()
        bare = from_frame(run.data, kind="four_wheel", meta={}, name="bare")
        self.assertEqual([len(a.lines) for a in figure([bare], marks=True).axes],
                         [len(a.lines) for a in figure([bare]).axes])
        error = io.StringIO()
        with tempfile.TemporaryDirectory() as directory, patch("rallytools.cli.load", return_value=bare), \
                contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(error):
            self.assertEqual(cli.main(["suspension", "x.csv", "--events", "--save", str(Path(directory) / "b.png")]), 0)
        self.assertIn("no wheel_radius", error.getvalue())

    def test_kind_colours_are_distinct_and_not_run_colours(self):
        from matplotlib.colors import to_hex
        from rallytools.charts.marks import KINDS
        colours = [to_hex(c) for c in KINDS.values()]
        self.assertEqual(len(set(colours)), len(colours))
        self.assertFalse(set(colours) & {to_hex(f"C{i}") for i in range(10)})
        self.assertEqual(set(KINDS), {"lock", "spin", "lift", "bump_stop", "droop_stop", "touchdown"})

    def test_cli_passes_events_to_the_chart(self):
        import rallytools.charts.suspension as suspension
        seen = []
        real = suspension.figure

        def spy(*args, **kwargs):
            seen.append(kwargs.get("marks"))
            return real(*args, **kwargs)

        fixture = str(FIXTURES / "four_wheel_springs.csv")
        with tempfile.TemporaryDirectory() as directory, patch.object(suspension, "figure", spy), \
                contextlib.redirect_stdout(io.StringIO()):
            cli.main(["suspension", fixture, "--events", "--save", str(Path(directory) / "e.png")])
            cli.main(["suspension", fixture, "--save", str(Path(directory) / "p.png")])
        self.assertEqual(seen, [True, False])

    def test_cli_events_need_a_chart(self):
        fixture = str(FIXTURES / "four_wheel_springs.csv")
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(cli.main(["suspension", fixture, "--events"]), 2)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "ev.png"
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli.main(["suspension", fixture, "--events", "--save", str(path)]), 0)
            self.assertTrue(path.exists())


if __name__ == "__main__":
    unittest.main()
