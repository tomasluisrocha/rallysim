import os
import subprocess
import sys
import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from rallytools import cli, paths
from rallytools.run_all import run_all
from . import support

EXE = os.environ.get("RALLYSIM_EXE")
TOOLS = Path(__file__).resolve().parent.parent


class RunAllTest(unittest.TestCase):
    def setUp(self):
        support.need(self, EXE and Path(EXE).exists(), "RALLYSIM_EXE not set to a built rallysim")

    def pictures(self, out):
        return sorted(p.relative_to(out).as_posix() for p in Path(out).rglob("*.png"))

    def test_only_two(self):
        with tempfile.TemporaryDirectory() as out:
            failed = run_all(EXE, ["fw_brake", "tyre_rig"], out=out)
            self.assertEqual(failed, [])
            self.assertTrue((Path(out) / "runs" / "fw_brake.csv").exists())
            self.assertTrue((Path(out) / "rig" / "tyre_rig.csv").exists())
            self.assertEqual(self.pictures(out), ["plots/tyres/fw_brake.png", "plots/tyres/tyre_rig.png",
                                                  "plots/views/fw_brake.png"])

    def test_no_plots(self):
        with tempfile.TemporaryDirectory() as out:
            self.assertEqual(run_all(EXE, ["fw_brake", "tyre_rig"], plots=False, out=out), [])
            self.assertTrue((Path(out) / "runs" / "fw_brake.csv").exists())
            self.assertTrue((Path(out) / "rig" / "tyre_rig.csv").exists())
            self.assertFalse((Path(out) / "plots").exists())

    def test_unknown_scenario(self):
        with tempfile.TemporaryDirectory() as out:
            with self.assertRaisesRegex(ValueError, "nope.*fw_brake"):
                run_all(EXE, ["nope"], out=out)

    def test_snapshot(self):
        with tempfile.TemporaryDirectory() as out:
            run_all(EXE, ["fw_brake"], plots=False, snapshot="s", out=out)
            self.assertTrue((Path(out) / "snapshots" / "s" / "runs" / "fw_brake.csv").exists())

    def test_failed_scenario(self):
        real = paths.run_command
        message = "spring_front: must be above 0"

        def fake(command, *args, **kwargs):
            if "fw_brake" in [str(c) for c in command[1:2]]:
                return False, message
            return real(command, *args, **kwargs)

        with tempfile.TemporaryDirectory() as out, mock.patch.object(paths, "run_command", fake):
            lines = []
            failed = run_all(EXE, ["fw_brake", "fw_lane"], plots=False, out=out, progress=lines.append)
            self.assertEqual(failed, ["fw_brake"])
            self.assertTrue((Path(out) / "runs" / "fw_lane.csv").exists())
            self.assertTrue(any("FAILED fw_brake" in line and message in line for line in lines), lines)
        with tempfile.TemporaryDirectory() as out, mock.patch.object(paths, "run_command", fake),                 mock.patch.object(cli, "OUT", Path(out)):
            printed = io.StringIO()
            with contextlib.redirect_stdout(printed):
                code = cli.main(["run-all", "--only", "fw_brake", "fw_lane", "--no-plots", "--exe", EXE])
            self.assertEqual(code, 1)
            self.assertIn("FAILED fw_brake", printed.getvalue())

    def test_pyplot_not_imported(self):
        # Pictures drawn through pyplot would leave it imported (and their figures open).
        code = ("import sys, tempfile\n"
                "from rallytools.run_all import run_all\n"
                "with tempfile.TemporaryDirectory() as out:\n"
                f"    run_all({EXE!r}, ['fw_brake'], out=out)\n"
                "sys.exit(1 if 'matplotlib.pyplot' in sys.modules else 0)\n")
        done = subprocess.run([sys.executable, "-c", code], cwd=TOOLS, capture_output=True, text=True)
        self.assertEqual(done.returncode, 0, done.stderr or "pyplot was imported")


if __name__ == "__main__":
    unittest.main()
