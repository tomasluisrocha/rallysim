import os
import sys
import tempfile
import unittest
from pathlib import Path

from rallytools import paths

NAMES = ("ROOT", "OUT", "RUNS", "RIG", "PLOTS", "VIEWS", "TYRES", "COMPARE", "SWEEP_PLOTS", "SWEEPS", "GIFS",
         "SNAPSHOTS", "SCRATCH", "WHEELS")


class TestPaths(unittest.TestCase):
    def test_layout(self):
        self.assertTrue((paths.ROOT / "tools" / "rally.py").exists())
        self.assertEqual(paths.RUNS, paths.ROOT / "out" / "runs")
        self.assertEqual(paths.SWEEP_PLOTS, paths.ROOT / "out" / "plots" / "sweeps")
        self.assertEqual(paths.GIFS, paths.ROOT / "out" / "gif")

    def test_simdata_shares_names(self):
        tools = str(paths.ROOT / "tools")
        added = tools not in sys.path
        if added:
            sys.path.insert(0, tools)
        try:
            import simdata
        finally:
            if added:
                sys.path.remove(tools)
        for name in NAMES:
            self.assertIs(getattr(simdata, name), getattr(paths, name), name)
        self.assertIs(simdata.label_of, paths.label_of)
        self.assertIs(simdata.run_command, paths.run_command)
        with tempfile.TemporaryDirectory() as empty:
            with self.assertRaises(SystemExit):
                simdata.find_exe(root=Path(empty))

    def test_find_exe(self):
        self.assertEqual(paths.find_exe("x/y"), Path("x/y"))
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            with self.assertRaises(ValueError):
                paths.find_exe(root=root)
            (root / "build").mkdir()
            (root / "build" / "rallysim").write_text("")
            self.assertEqual(paths.find_exe(root=root), root / "build" / "rallysim")

    def test_release_before_debug(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            for config in ("Debug", "Release"):
                (root / "build" / config).mkdir(parents=True)
                (root / "build" / config / "rallysim.exe").write_text("")
            self.assertEqual(paths.find_exe(root=root), root / "build" / "Release" / "rallysim.exe")

    def test_run_command_from_the_sim_folder(self):
        # called from another folder, the command still runs in the sim folder
        here = os.getcwd()
        with tempfile.TemporaryDirectory() as folder:
            os.chdir(folder)
            try:
                ok, text = paths.run_command([sys.executable, "-c", "import os; print(os.getcwd())"])
            finally:
                os.chdir(here)
        self.assertTrue(ok)
        self.assertEqual(Path(text).resolve(), paths.ROOT)

    def test_label_of(self):
        self.assertEqual(paths.label_of(paths.SNAPSHOTS / "linear" / "runs" / "fw_brake.csv"), "linear/fw_brake")
        self.assertEqual(paths.label_of("a/b/fw_lane.csv"), "fw_lane")

    def test_run_command(self):
        self.assertEqual(paths.run_command([sys.executable, "-c", "print(1)"]), (True, "1"))


if __name__ == "__main__":
    unittest.main()
