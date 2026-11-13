import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import matplotlib.pyplot as plt
from matplotlib.figure import Figure

from rallytools import cli, tyre
from rallytools.charts.tyre_points import figure
from rallytools.run import load
from .support import need

FIXTURES = Path(__file__).parent / "fixtures"


class TestTyreByHand(unittest.TestCase):
    def test_grip(self):
        t = dict(tyre.TYRE_DEFAULTS)
        self.assertAlmostEqual(float(tyre.grip(t, 3000.0)), 3000.0, places=9)
        self.assertAlmostEqual(float(tyre.grip(t, 6000.0)), 5400.0, places=9)  # mu 0.9 at twice the nominal load
        self.assertEqual(float(tyre.grip(t, 0.0)), 0.0)

    def test_linear_forces(self):
        t = dict(tyre.TYRE_DEFAULTS)
        fx, fy = tyre.tyre_forces(t, 0.01, 0.0, 3000.0)
        self.assertAlmostEqual(float(fx), 900.0, places=9)
        self.assertAlmostEqual(float(fy), 0.0, places=9)
        fx, fy = tyre.tyre_forces(t, 0.5, 0.0, 3000.0)  # past the limit: the force is the grip
        self.assertAlmostEqual(float(fx), 3000.0, places=9)
        self.assertAlmostEqual(float(fy), 0.0, places=9)


class TestRig(unittest.TestCase):
    def test_copy_matches_cpp(self):
        executable = os.environ.get("RALLYSIM_EXE")
        need(self, executable and os.path.isfile(executable),
             "RALLYSIM_EXE must name an existing rallysim executable")
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as directory:
            rig = Path(directory) / "rig.csv"
            result = subprocess.run([executable, "tyre_rig", str(rig)], cwd=root, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertLess(tyre.check_against_rig(rig), 1.0)


class TestTyrePointsChart(unittest.TestCase):
    def test_axes_and_lines(self):
        chart = figure(load(FIXTURES / "four_wheel_springs.csv"))
        self.assertIsInstance(chart, Figure)
        self.assertEqual([len(axis.lines) for axis in chart.axes], [7, 1, 7, 4])

    def test_show_draws_into_a_pyplot_figure(self):
        plt.close("all")
        pyplot_figure = plt.figure

        def as_in_matplotlib_3_10(*args, **kwargs):
            # matplotlib 3.10 (the CI floor) refuses a figure pyplot did not make; 3.11 adopts it.
            if args and isinstance(args[0], Figure) and args[0].canvas.manager is None:
                raise ValueError("The passed figure is not managed by pyplot")
            return pyplot_figure(*args, **kwargs)

        with patch("matplotlib.pyplot.figure", side_effect=as_in_matplotlib_3_10),                 patch("matplotlib.pyplot.show") as show:
            self.assertEqual(cli.main(["tyre-points", str(FIXTURES / "four_wheel_springs.csv")]), 0)
        self.assertEqual(len(plt.get_fignums()), 1)
        self.assertEqual(len(plt.gcf().axes), 4)
        show.assert_called_once_with()
        plt.close("all")

    def test_figure_draws_into_a_given_figure(self):
        target = Figure()
        self.assertIs(figure(load(FIXTURES / "four_wheel_springs.csv"), fig=target), target)
        self.assertEqual(len(target.axes), 4)

    def test_other_kinds_raise(self):
        with self.assertRaises(ValueError):
            figure(load(FIXTURES / "bicycle.csv"))


class TestImport(unittest.TestCase):
    def test_no_matplotlib(self):
        root = Path(__file__).resolve().parents[1]
        code = "import sys, rallytools.tyre; sys.exit(1 if 'matplotlib' in sys.modules else 0)"
        result = subprocess.run([sys.executable, "-c", code], cwd=root, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == "__main__":
    unittest.main()
