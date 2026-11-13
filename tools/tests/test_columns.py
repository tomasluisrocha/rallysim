import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
import numpy as np
import pandas as pd

from rallytools import cli
from rallytools.charts.columns import figure
from rallytools.run import from_frame, load

FIXTURES = Path(__file__).parent / "fixtures"
KINDS = ("bicycle", "rigid_body", "single_track", "four_wheel_springs")


class TestColumnsChart(unittest.TestCase):
    def tearDown(self):
        plt.close("all")

    def test_one_axes_with_a_line_per_column(self):
        chart = figure(load(FIXTURES / "bicycle.csv"), "time", ["yaw", "speed"])
        self.assertEqual(len(chart.axes), 1)
        self.assertEqual([line.get_label() for line in chart.axes[0].get_lines()], ["yaw", "speed"])
        self.assertEqual(chart.axes[0].get_xlabel(), "time")
        self.assertEqual(chart.axes[0].get_title(), "bicycle")

    def test_yaw_is_unwrapped_unless_wrapped(self):
        yaw = [3.0, 3.1, -3.1, -3.0]
        run = from_frame(pd.DataFrame({"time": [0.0, 1.0, 2.0, 3.0], "yaw": yaw,
                                       "speed": [1.0, 1.0, 1.0, 1.0]}), name="wrap")
        drawn = figure(run, "time", ["yaw"]).axes[0].get_lines()[0].get_ydata()
        expected = [3.0, 3.1, 2 * np.pi - 3.1, 2 * np.pi - 3.0]
        np.testing.assert_allclose(drawn, expected, atol=1e-12)
        kept = figure(run, "time", ["yaw"], wrapped=True).axes[0].get_lines()[0].get_ydata()
        np.testing.assert_allclose(kept, yaw, atol=1e-12)

    def test_missing_column_is_named_with_the_available_ones(self):
        with self.assertRaises(ValueError) as caught:
            figure(load(FIXTURES / "bicycle.csv"), "time", ["nope"])
        self.assertIn("nope", str(caught.exception))
        self.assertIn("speed", str(caught.exception))

    def test_every_kind_draws_time_against_its_second_column(self):
        for kind in KINDS:
            with self.subTest(kind=kind):
                run = load(FIXTURES / (kind + ".csv"))
                second = run.data.columns[1]
                line = figure(run, "time", [second]).axes[0].get_lines()[0]
                self.assertEqual(line.get_label(), second)
                np.testing.assert_allclose(line.get_xdata(), run.column("time"))

    def test_draws_into_a_given_figure(self):
        target = Figure()
        self.assertIs(figure(load(FIXTURES / "bicycle.csv"), "time", ["yaw"], fig=target), target)
        self.assertEqual(len(target.axes), 1)


class TestColumnsCommand(unittest.TestCase):
    def tearDown(self):
        plt.close("all")

    def test_save_and_unknown_column(self):
        fixture = str(FIXTURES / "bicycle.csv")
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "c.png"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(["columns", fixture, "time", "yaw", "--save", str(output)]), 0)
            self.assertTrue(output.exists())
            error = io.StringIO()
            with contextlib.redirect_stderr(error):
                self.assertEqual(cli.main(["columns", fixture, "time", "nope", "--save", str(output)]), 2)
            self.assertIn("nope", error.getvalue())

    def test_cli_passes_wrapped(self):
        import rallytools.charts.columns as columns
        seen = []
        real = columns.figure

        def spy(*args, **kwargs):
            seen.append(args[3] if len(args) > 3 else kwargs.get("wrapped", False))
            return real(*args, **kwargs)

        fixture = str(FIXTURES / "bicycle.csv")
        with tempfile.TemporaryDirectory() as folder, patch.object(columns, "figure", spy), \
                contextlib.redirect_stdout(io.StringIO()):
            cli.main(["columns", fixture, "time", "yaw", "--wrapped", "--save", str(Path(folder) / "w.png")])
            cli.main(["columns", fixture, "time", "yaw", "--save", str(Path(folder) / "u.png")])
        self.assertEqual(seen, [True, False])

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
            self.assertEqual(cli.main(["columns", str(FIXTURES / "bicycle.csv"), "time", "yaw"]), 0)
        self.assertEqual(len(plt.get_fignums()), 1)
        self.assertEqual(len(plt.gcf().axes), 1)
        show.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
