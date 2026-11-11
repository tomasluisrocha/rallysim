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


if __name__ == "__main__":
    unittest.main()
