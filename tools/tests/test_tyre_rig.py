import os
import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from rallytools import cli  # noqa: E402
from rallytools.charts.tyre_rig import figure  # noqa: E402

from .support import need  # noqa: E402


class TyreRigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.path = None
        exe = os.environ.get("RALLYSIM_EXE")
        if exe and Path(exe).exists():
            target = Path(cls.tmp.name) / "tyre_rig.csv"
            done = subprocess.run([exe, "tyre_rig", str(target)], capture_output=True, text=True)
            if done.returncode == 0 and target.exists():
                cls.path = str(target)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def setUp(self):
        need(self, self.path is not None, "needs a built rallysim (RALLYSIM_EXE)")

    def test_figure_has_four_panels(self):
        chart = figure(self.path)
        self.assertIsInstance(chart, Figure)
        self.assertEqual(len(chart.axes), 4)
        self.assertEqual([len(ax.lines) for ax in chart.axes], [18, 18, 10, 7])
        self.assertEqual(chart.axes[0].get_title(), "Pure longitudinal slip")
        self.assertEqual(chart.axes[1].get_title(), "Pure lateral slip")
        # the combined panels show the car's curve, tmeasy, though the file lists linear first
        self.assertIn("(tmeasy)", chart.axes[2].get_title())
        self.assertIn("(tmeasy)", chart.axes[3].get_title())
        self.assertIn("(peak_slide)", figure(self.path, curves=["peak_slide", "tmeasy"]).axes[2].get_title())

    def test_curves_select_and_unknown_is_refused(self):
        full = figure(self.path)
        one = figure(self.path, curves=["tmeasy"])
        self.assertLess(len(one.axes[0].lines), len(full.axes[0].lines))
        with self.assertRaises(ValueError) as caught:
            figure(self.path, curves=["nope"])
        self.assertIn("tmeasy", str(caught.exception))

    def test_save_writes_a_png(self):
        output = Path(self.tmp.name) / "r.png"
        self.assertEqual(cli.main(["tyre-rig", self.path, "--save", str(output)]), 0)
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
            self.assertEqual(cli.main(["tyre-rig", self.path]), 0)
        self.assertEqual(len(plt.get_fignums()), 1)
        self.assertEqual(len(plt.gcf().axes), 4)
        show.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
