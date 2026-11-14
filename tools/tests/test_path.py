import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.figure import Figure

from rallytools import cli
from rallytools.charts.path import figure
from rallytools.overhead import view_of
from rallytools.run import load

FIXTURES = Path(__file__).parent / "fixtures"
KINDS = {"rigid_body": 2, "bicycle": 2, "single_track": 3, "four_wheel_springs": 3}
TOOLS = Path(__file__).resolve().parents[1]


def time_labels(chart):
    """The car-time labels drawn on the picture, in seconds."""
    return [float(text.get_text().split()[0]) for text in chart.axes[0].texts if text.get_text().endswith(" s")]


class TestPathChart(unittest.TestCase):
    def tearDown(self):
        plt.close("all")

    def test_axes_and_size_for_each_kind(self):
        for name, count in KINDS.items():
            with self.subTest(name):
                chart = figure(load(FIXTURES / f"{name}.csv"))
                self.assertEqual(len(chart.axes), count)
                self.assertEqual(tuple(chart.get_size_inches()), (12.0, 7.5))

    def test_path_line_is_the_run_position(self):
        for name in KINDS:
            with self.subTest(name):
                run = load(FIXTURES / f"{name}.csv")
                ax = figure(run).axes[0]
                path = [c for c in ax.collections if isinstance(c, LineCollection)][0]
                segments = np.array(path.get_segments())
                np.testing.assert_allclose(segments[:, 0, 0], run.column("pos.x")[:-1])
                np.testing.assert_allclose(segments[:, 0, 1], run.column("pos.y")[:-1])
                self.assertEqual(segments[-1, 1, 0], run.column("pos.x")[-1])
                self.assertEqual(segments[-1, 1, 1], run.column("pos.y")[-1])
                np.testing.assert_allclose(path.get_array(), view_of(run).speed[:-1])

    def test_cars_are_drawn_at_the_times_every_gives(self):
        run = load(FIXTURES / "four_wheel_springs.csv")
        view = view_of(run)
        for every in (0.05, 0.02):
            with self.subTest(every=every):
                times = np.arange(view.t[0], view.t[-1] + 1e-9, every)
                rows = np.clip(np.searchsorted(view.t, times), 0, len(view.t) - 1)
                expected = [float(f"{view.t[i]:.1f}") for i in rows]
                self.assertEqual(time_labels(figure(run, every)), expected)

    def test_a_smaller_interval_draws_more_cars(self):
        run = load(FIXTURES / "four_wheel_springs.csv")
        coarse = len(figure(run, 0.05).axes[0].patches)
        fine = len(figure(run, 0.02).axes[0].patches)
        self.assertGreater(fine, coarse)

    def test_default_interval_gives_about_ten_cars(self):
        run = load(FIXTURES / "four_wheel_springs.csv")
        view = view_of(run)
        every = max(round((view.t[-1] - view.t[0]) / 10, 1), 0.1)
        self.assertIn(f"the car every {every:g} s", figure(run).axes[0].get_title())

    def test_draws_into_a_given_figure(self):
        target = Figure()
        self.assertIs(figure(load(FIXTURES / "bicycle.csv"), fig=target), target)
        self.assertEqual(len(target.axes), 2)

    def test_save_writes_a_png_for_each_kind(self):
        with tempfile.TemporaryDirectory() as folder:
            for name in KINDS:
                with self.subTest(name):
                    output = Path(folder) / f"{name}.png"
                    self.assertEqual(cli.main(["path", str(FIXTURES / f"{name}.csv"), "--save", str(output)]), 0)
                    self.assertGreater(output.stat().st_size, 10_000)

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
            self.assertEqual(cli.main(["path", str(FIXTURES / "four_wheel_springs.csv")]), 0)
        self.assertEqual(len(plt.get_fignums()), 1)
        self.assertEqual(len(plt.gcf().axes), 3)
        show.assert_called_once_with()

    def test_import_leaves_out_the_viewer_libraries(self):
        code = ("import sys; import rallytools.charts.path; "
                "sys.exit(1 if 'PySide6' in sys.modules or 'pyvista' in sys.modules else 0)")
        result = subprocess.run([sys.executable, "-c", code], cwd=TOOLS, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
