"""The old scripts in tools/ are short forms of rally.py commands. Each is checked for --help, for what it hands
on, and (where it saves a picture) for one real run."""
import contextlib
import io
import os
import runpy
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.figure import Figure

from rallytools.charts import path as path_chart
from . import support

TOOLS = Path(__file__).resolve().parents[1]
SIM = TOOLS.parent
FIXTURES = Path(__file__).parent / "fixtures"
EXE = os.environ.get("RALLYSIM_EXE")
BRAKE = str(FIXTURES / "four_wheel_springs.csv")
OTHER = str(FIXTURES / "four_wheel_unsprung.csv")


def run_script(name, *argv):
    """Run tools/<name> in this process as a script; the exit code (0 when it just returns)."""
    with patch.object(sys, "argv", [str(TOOLS / name), *argv]), contextlib.redirect_stdout(io.StringIO()):
        try:
            runpy.run_path(str(TOOLS / name), run_name="__main__")
        except SystemExit as stop:
            return 0 if stop.code in (None, 0) else stop.code
    return 0


def run_process(folder, name, *argv):
    env = dict(os.environ, MPLBACKEND="Agg")
    return subprocess.run([sys.executable, str(Path(folder) / name), *argv], cwd=folder, env=env,
                          capture_output=True, text=True, timeout=300)


def copy_of_tools(folder):
    """tools/ copied to <folder>/sim/tools, so ROOT and out/ are in the folder and the real out/ is left alone."""
    target = Path(folder) / "sim"
    shutil.copytree(TOOLS, target / "tools", ignore=shutil.ignore_patterns("__pycache__", "tests"))
    return target


class Helpful:
    """Mixin: every script answers --help with exit 0."""
    script = ""

    def test_help(self):
        result = run_process(SIM, f"tools/{self.script}", "--help")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("usage", result.stdout.lower())


class ForwardingWrapper(Helpful):
    """Mixin: a script that hands its arguments to rallytools.cli.main."""

    def forwarded(self, *argv):
        with patch("rallytools.cli.main", return_value=0) as main:
            self.assertEqual(run_script(self.script, *argv), 0)
        main.assert_called_once()
        return main.call_args.args[0]


class PlotRunTest(ForwardingWrapper, unittest.TestCase):
    script = "plot_run.py"

    def test_wrapped_is_forwarded_only_when_given(self):
        self.assertEqual(self.forwarded(BRAKE, "time", "speed"), ["columns", BRAKE, "time", "speed"])
        self.assertEqual(self.forwarded(BRAKE, "time", "yaw", "--wrapped"),
                         ["columns", BRAKE, "time", "yaw", "--wrapped"])

    def test_saves_a_picture(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "columns.png"
            result = run_process(SIM, "tools/plot_run.py", BRAKE, "time", "speed", "--save", str(output))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertGreater(output.stat().st_size, 10_000)


class TyrePointsTest(ForwardingWrapper, unittest.TestCase):
    script = "tyre_points.py"

    def test_calls_tyre_points_not_tyre_rig(self):
        self.assertEqual(self.forwarded(BRAKE, "--save", "--min-speed", "2"),
                         ["tyre-points", BRAKE, "--save", "--min-speed", "2"])

    def test_saves_a_picture(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "points.png"
            result = run_process(SIM, "tools/tyre_points.py", BRAKE, "--save", str(output))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertGreater(output.stat().st_size, 10_000)


class TyrePlotTest(ForwardingWrapper, unittest.TestCase):
    script = "tyre_plot.py"

    def test_calls_tyre_rig(self):
        self.assertEqual(self.forwarded("rig.csv", "--curve", "peak_slide"),
                         ["tyre-rig", "rig.csv", "--curve", "peak_slide"])

    def test_saves_a_picture(self):
        support.need(self, EXE and Path(EXE).exists(), "RALLYSIM_EXE not set to a built rallysim")
        with tempfile.TemporaryDirectory() as folder:
            rig = Path(folder) / "tyre_rig.csv"
            made = subprocess.run([EXE, "tyre_rig", str(rig)], capture_output=True, text=True, timeout=300)
            self.assertEqual(made.returncode, 0, made.stderr)
            output = Path(folder) / "rig.png"
            result = run_process(SIM, "tools/tyre_plot.py", str(rig), "--save", str(output))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertGreater(output.stat().st_size, 10_000)


class SweepTest(ForwardingWrapper, unittest.TestCase):
    script = "sweep.py"

    def test_calls_sweep(self):
        self.assertEqual(self.forwarded("fw_brake", "mu_front", "0.8", "1.0"),
                         ["sweep", "fw_brake", "mu_front", "0.8", "1.0"])

    def test_saves_a_picture(self):
        support.need(self, EXE and Path(EXE).exists(), "RALLYSIM_EXE not set to a built rallysim")
        with tempfile.TemporaryDirectory() as folder:
            sim = copy_of_tools(folder)
            output = Path(folder) / "sweep.png"
            result = run_process(sim, "tools/sweep.py", "fw_brake", "mu_front", "0.8", "1.0", "--exe", EXE,
                                 "--jobs", "2", "--save", str(output))
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertGreater(output.stat().st_size, 10_000)


class RunAllTest(ForwardingWrapper, unittest.TestCase):
    script = "run_all.py"

    def test_calls_run_all(self):
        self.assertEqual(self.forwarded("--only", "fw_brake", "--no-plots"),
                         ["run-all", "--only", "fw_brake", "--no-plots"])

    def test_makes_a_picture(self):
        support.need(self, EXE and Path(EXE).exists(), "RALLYSIM_EXE not set to a built rallysim")
        with tempfile.TemporaryDirectory() as folder:
            sim = copy_of_tools(folder)
            result = run_process(sim, "tools/run_all.py", "--only", "fw_brake", "--exe", EXE, "--jobs", "2")
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertGreater((sim / "out" / "plots" / "views" / "fw_brake.png").stat().st_size, 10_000)


class CompareTest(Helpful, unittest.TestCase):
    script = "compare.py"

    def layout(self, folder):
        root = Path(folder).resolve()
        (root / "snapshots" / "snap" / "runs").mkdir(parents=True)
        (root / "runs").mkdir()
        shutil.copy(BRAKE, root / "snapshots" / "snap" / "runs" / "x.csv")
        shutil.copy(OTHER, root / "runs" / "x.csv")
        return root

    def test_against_pairs_the_snapshot_as_a_and_the_current_run_as_b(self):
        with tempfile.TemporaryDirectory() as folder:
            root = self.layout(folder)
            with patch("rallytools.paths.SNAPSHOTS", root / "snapshots"), patch("rallytools.paths.RUNS", root / "runs"), \
                    patch("rallytools.paths.COMPARE", root / "compare"), \
                    patch("rallytools.charts.overlay.table", return_value="table") as table, \
                    patch("rallytools.charts.overlay.figure", side_effect=lambda *a, **k: Figure()) as figure:
                self.assertEqual(run_script("compare.py", "--against", "snap", "x", "--save"), 0)
            table.assert_called_once()
            self.assertEqual(table.call_args.kwargs["paths"], [root / "snapshots" / "snap" / "runs" / "x.csv",
                                                               root / "runs" / "x.csv"])
            runs, _, labels = table.call_args.args
            self.assertEqual(labels, ("snap", "current"))
            self.assertEqual([run.name for run in runs], ["x", "x"])
            self.assertEqual(figure.call_args.args[2], ("snap", "current"))
            self.assertEqual(len(list((root / "compare").glob("x__snap_vs_current.png"))), 1)

    def test_files_are_a_then_b(self):
        with patch("rallytools.charts.overlay.table", return_value="table") as table, \
                patch("rallytools.charts.overlay.figure", side_effect=lambda *a, **k: Figure()), \
                tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "c.png"
            self.assertEqual(run_script("compare.py", BRAKE, OTHER, "--save", str(output)), 0)
        self.assertEqual(table.call_args.kwargs["paths"], [Path(BRAKE), Path(OTHER)])

    def test_saves_a_picture(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "compare.png"
            result = run_process(SIM, "tools/compare.py", BRAKE, OTHER, "--save", str(output))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertGreater(output.stat().st_size, 10_000)


class TopdownStaticTest(Helpful, unittest.TestCase):
    script = "topdown.py"

    def test_every_is_passed_through(self):
        with tempfile.TemporaryDirectory() as folder, \
                patch("rallytools.charts.path.figure", wraps=path_chart.figure) as figure:
            output = Path(folder) / "static.png"
            self.assertEqual(run_script("topdown.py", BRAKE, "--static", "--every", "0.5", "--save", str(output)), 0)
        figure.assert_called_once()
        self.assertEqual(figure.call_args.args[1], 0.5)

    def test_saves_a_picture(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "static.png"
            result = run_process(SIM, "tools/topdown.py", BRAKE, "--static", "--save", str(output))
            self.assertEqual(result.returncode, 0, result.stderr)
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
                patch("matplotlib.pyplot.show") as show,                 patch("rallytools.charts.path.figure", wraps=path_chart.figure) as figure:
            self.assertEqual(run_script("topdown.py", BRAKE, "--static", "--every", "0.5"), 0)
        self.assertEqual(figure.call_args.args[1], 0.5)
        self.assertEqual(len(plt.get_fignums()), 1)
        self.assertEqual(len(plt.gcf().axes), 3)
        show.assert_called_once_with()
        plt.close("all")


if __name__ == "__main__":
    unittest.main()
