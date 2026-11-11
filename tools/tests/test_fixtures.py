import os
from pathlib import Path
import shutil
import subprocess
import unittest

from rallytools import run


FIXTURES = Path(__file__).parent / "fixtures"
EXPECTED = {
    "rigid_body": ("rigid_body", 6, 0.001, 100, 21),
    "bicycle": ("bicycle", 6, 0.001, 100, 11),
    "single_track": ("single_track", 6, 0.001, 100, 24),
    "four_wheel_springs": ("four_wheel", 6, 0.001, 100, 101),
    "four_wheel_unsprung": ("four_wheel", 6, 0.001, 100, 101),
    "four_wheel_schema5": ("four_wheel", 5, 0.001, 100, 89),
}


class TestFixtures(unittest.TestCase):
    def test_shape(self):
        for name, expected in EXPECTED.items():
            with self.subTest(name=name):
                fixture = run.load(FIXTURES / f"{name}.csv")
                kind, schema, dt, rows, columns = expected
                self.assertEqual(fixture.kind, kind)
                self.assertEqual(fixture.schema, schema)
                self.assertEqual(fixture.dt, dt)
                self.assertEqual(len(fixture.data), rows)
                self.assertEqual(len(fixture.data.columns), columns)

    def test_vertical(self):
        self.assertEqual(run.load(FIXTURES / "four_wheel_springs.csv").vertical, "springs")
        self.assertEqual(run.load(FIXTURES / "four_wheel_unsprung.csv").vertical, "unsprung")
        self.assertEqual(run.load(FIXTURES / "four_wheel_schema5.csv").vertical, "springs")

    def test_unix_line_endings(self):
        for path in FIXTURES.glob("*.csv"):
            with self.subTest(path=path.name):
                self.assertNotIn(b"\r", path.read_bytes())

    def test_springs_and_unsprung_are_distinct(self):
        unsprung = run.load(FIXTURES / "four_wheel_unsprung.csv")
        springs = run.load(FIXTURES / "four_wheel_springs.csv")
        self.assertGreater(unsprung.column("wheel.FL.wheelZ")[0], 0)
        self.assertEqual(springs.column("wheel.FL.wheelZ")[0], 0)

    def test_gitignore_exception(self):
        if shutil.which("git") is None:
            self.skipTest("git is not on PATH")
        root = FIXTURES.parents[2]
        tracked_fixture = subprocess.run(
            ["git", "check-ignore", "tools/tests/fixtures/bicycle.csv"],
            cwd=root,
            capture_output=True,
        )
        ignored_csv = subprocess.run(
            ["git", "check-ignore", "tools/x.csv"],
            cwd=root,
            capture_output=True,
        )
        self.assertEqual(tracked_fixture.returncode, 1)
        self.assertEqual(ignored_csv.returncode, 0)


if __name__ == "__main__":
    unittest.main()
