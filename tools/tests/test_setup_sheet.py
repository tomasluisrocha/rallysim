import contextlib
import io
import unittest

import pandas as pd

from rallytools import run, setup_sheet


def make_run(name, **meta):
    data = pd.DataFrame({"step": [0], "time": [0.0], "yaw": [0.0], "speed": [1.0]})
    return run.from_frame(data, kind="bicycle", name=name, meta=meta)


class TestSetupSheet(unittest.TestCase):
    def test_precision_and_non_adjacent_difference(self):
        runs = [make_run("a", mass=1200), make_run("b", mass=1200), make_run("c", mass=1200.0001)]
        row = setup_sheet.rows(runs)[0]
        self.assertTrue(row[2])
        self.assertIn("1200.0001", setup_sheet.text(runs))

        runs = [make_run("a", value=1), make_run("b", value=2), make_run("c", value=1)]
        self.assertTrue(setup_sheet.rows(runs)[0][2])

    def test_unchanged_filtering_and_marker(self):
        runs = [make_run("a", mass=1200, spring_front=50000),
                make_run("b", mass=1200, spring_front=50000)]
        output = setup_sheet.text(runs, changed_only=True)
        self.assertEqual(output, "\tkey\ta\tb")
        self.assertNotIn("spring_front", output)
        self.assertNotIn("mass", output)

    def test_missing_key(self):
        runs = [make_run("a", mass=1200), make_run("b", mass=1200),
                make_run("c", mass=1200, vertical=1.0)]
        output = setup_sheet.text(runs)
        self.assertIn("*\tvertical\t-\t-\t1.0", output)

    def test_key_order(self):
        runs = [make_run("a", z=1, a=2), make_run("b", z=1, a=2, m=3, n=4)]
        self.assertEqual([row[0] for row in setup_sheet.rows(runs)], ["z", "a", "m", "n"])

    def test_cli_exit_codes(self):
        from rallytools import cli
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli.main(["setup", "missing.csv"]), 2)


if __name__ == "__main__":
    unittest.main()
