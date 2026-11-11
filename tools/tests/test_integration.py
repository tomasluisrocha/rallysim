import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from rallytools import required, run
from .support import need


class TestIntegration(unittest.TestCase):
    def test_required_matches_fixture(self):
        fixture = run.load(Path(__file__).parent / "fixtures" / "four_wheel_springs.csv")
        self.assertEqual(len(required.FOUR_WHEEL), 101)
        self.assertEqual(set(required.FOUR_WHEEL), set(fixture.data.columns))
        self.assertEqual(required.FOUR_WHEEL, fixture.units)

    def test_fw_launch_contract(self):
        executable = os.environ.get("RALLYSIM_EXE")
        need(self, executable and os.path.isfile(executable),
             "RALLYSIM_EXE must name an existing rallysim executable")

        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "fw_launch.csv"
            result = subprocess.run(
                [executable, "fw_launch", str(output)],
                cwd=root,
                capture_output=True,
                text=True,
            )
            if result.returncode:
                self.fail(f"fw_launch failed ({result.returncode}): {result.stderr}")
            telemetry = run.load(output)

        self.assertEqual(telemetry.schema, required.SCHEMA)
        self.assertEqual(len(telemetry.data), 3501)
        for column, unit in required.FOUR_WHEEL.items():
            with self.subTest(column=column):
                self.assertIn(column, telemetry.data.columns)
                self.assertEqual(telemetry.units.get(column), unit)


if __name__ == "__main__":
    unittest.main()
