import os
from pathlib import Path
import subprocess
import tempfile
import unittest

import numpy as np

import view3d
from . import support

EXE = os.environ.get("RALLYSIM_EXE")
ROOT = Path(__file__).resolve().parents[2]


class TestRideHeight(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.folder = tempfile.TemporaryDirectory()
        cls.path = Path(cls.folder.name) / "fw_drop.csv"
        if EXE and Path(EXE).exists():
            made = subprocess.run([EXE, "fw_drop", str(cls.path)], capture_output=True, text=True,
                                  timeout=300, cwd=ROOT)
            if made.returncode:
                raise RuntimeError(made.stderr)

    @classmethod
    def tearDownClass(cls):
        cls.folder.cleanup()

    def setUp(self):
        support.need(self, EXE and Path(EXE).exists(), "RALLYSIM_EXE not set to a built rallysim")

    def test_exaggerated_about_the_rest_height(self):
        # Defect: exaggerating about the first frame draws the fall from 0.8 m as if it were the ride height's change.
        run = view3d.Run(str(self.path), 4)
        rest = run.cg_height
        self.assertAlmostEqual(run.z[0], rest + 4 * (0.8 - rest), delta=1e-9)
        lowest = run.z_true.min()
        self.assertAlmostEqual(run.z[int(np.argmin(run.z_true))], rest - 4 * (rest - lowest), delta=1e-9)

    def test_factor_one_is_true_scale(self):
        # Defect: the factor applied twice (or to the wrong base) moves the body at true scale.
        run = view3d.Run(str(self.path), 1)
        self.assertTrue(np.array_equal(run.z, run.z_true))


if __name__ == "__main__":
    unittest.main()
