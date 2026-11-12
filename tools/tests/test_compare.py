import contextlib
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import pandas as pd

from rallytools import compare, run
from .support import write_run


def frame(times, **columns):
    return pd.DataFrame({"step": np.arange(len(times)), "time": times, **columns})


class TestCompare(unittest.TestCase):
    def test_exact_and_largest(self):
        times = np.arange(1000) * 0.001
        a = run.from_frame(frame(times, speed=np.ones(1000), value=np.zeros(1000)), kind="bicycle", dt=.001)
        b_data = a.data.copy()
        b_data["value"] = np.where(times < .25, 0, np.where(times <= .9,
                              1e-6 + (times - .25) * 4e-6 / .65, 0))
        b = run.from_frame(b_data, kind="bicycle", dt=.001)
        result = compare.diff(a, b, [("*", compare.Tolerance(0, 0))])
        value = next(column for column in result.columns if column.name == "value")
        self.assertEqual(value.t_first, .25)
        self.assertAlmostEqual(value.largest, 5e-6, delta=1e-15)
        self.assertAlmostEqual(value.t_largest, .9)
        self.assertTrue(value.beyond)
        self.assertFalse(compare.diff(a, a, [("*", compare.Tolerance(0, 0))]).differs())

    def test_absolute_and_relative_tolerance(self):
        times = np.array([0., 1.])
        zero_a = run.from_frame(frame(times, value=[0., 0.]), kind="bicycle")
        zero_b = run.from_frame(frame(times, value=[.5, .5]), kind="bicycle")
        self.assertFalse(compare.diff(zero_a, zero_b, [("value", compare.Tolerance(.5, 0))]).differs())
        zero_b.data.loc[0, "value"] = .5000001
        self.assertTrue(compare.diff(zero_a, zero_b, [("value", compare.Tolerance(.5, 0))]).differs())
        a = run.from_frame(frame(times, value=[100., 100.]), kind="bicycle")
        b = run.from_frame(frame(times, value=[102., 102.]), kind="bicycle")
        self.assertFalse(compare.diff(a, b, [("value", compare.Tolerance(0, .0197))]).differs())
        # The larger of the two values is used, whichever run holds it.
        self.assertFalse(compare.diff(b, a, [("value", compare.Tolerance(0, .0197))]).differs())

    def test_rules_are_last_match_and_wildcards(self):
        times = np.array([0., 1.])
        a = run.from_frame(frame(times, **{"wheel.FL.Fz": [0, 0], "wheel.FL.Fx": [0, 0]}),
                           kind="four_wheel")
        b = run.from_frame(frame(times, **{"wheel.FL.Fz": [.5, .5], "wheel.FL.Fx": [.5, .5]}),
                           kind="four_wheel")
        rules = [("*", compare.Tolerance(1e-9)), ("wheel.*.Fz", compare.Tolerance(1))]
        result = compare.diff(a, b, rules)
        self.assertFalse(next(c for c in result.columns if c.name.endswith("Fz")).beyond)
        self.assertTrue(next(c for c in result.columns if c.name.endswith("Fx")).beyond)
        result = compare.diff(a, b, list(reversed(rules)))
        self.assertTrue(next(c for c in result.columns if c.name.endswith("Fz")).beyond)
        # "*" matches across the dots of a name.
        self.assertFalse(compare.diff(a, b, [("*", compare.Tolerance(1))]).differs())

    def test_alignment_wrap_missing_and_end(self):
        a_times = np.arange(1000) * .001
        b_times = np.arange(1999) * .0005
        a = run.from_frame(frame(a_times, value=a_times ** 2), kind="bicycle", dt=.001)
        b = run.from_frame(frame(b_times, value=b_times ** 2), kind="bicycle", dt=.0005)
        self.assertTrue(all(column.largest == 0 for column in compare.diff(a, b).columns))
        yaw_a = run.from_frame(frame([0.], yaw=[3.14], speed=[1.]), kind="bicycle")
        yaw_b = run.from_frame(frame([0.], yaw=[-3.14], speed=[1.]), kind="bicycle")
        self.assertAlmostEqual(compare.diff(yaw_a, yaw_b).columns[0].largest, .0031853, delta=1e-7)
        same_time = run.from_frame(frame([0.], value=[0.], extra=[1.]), kind="bicycle")
        same_a = run.from_frame(frame([0.], value=[0.]), kind="bicycle")
        result = compare.diff(same_a, same_time)
        self.assertIn("extra", result.only_b)
        self.assertFalse(result.differs(ignore_missing=True))
        self.assertIn("extra", compare.diff(same_time, same_a).only_a)
        self.assertTrue(compare.diff(same_time, same_a).differs())
        # B stops at 0.5 s and equals A up to there: nothing beyond, yet the runs differ in length.
        short = run.from_frame(a.data.iloc[:501].copy(), kind="bicycle", dt=.001)
        result = compare.diff(a, short)
        self.assertFalse(any(column.beyond for column in result.columns))
        self.assertEqual(result.end_b, .5)
        self.assertTrue(result.differs())

    def test_yaw_on_other_times(self):
        # The same heading on two time steps, passing +pi at 0.283 s.
        def heading(dt, rows):
            times = np.arange(rows) * dt
            yaw = (3.0 + 0.5 * times + np.pi) % (2 * np.pi) - np.pi
            return run.from_frame(frame(times, yaw=yaw, speed=np.ones(rows)), kind="bicycle", dt=dt)
        result = compare.diff(heading(.001, 1001), heading(.0008, 1251))
        self.assertLess(next(c for c in result.columns if c.name == "yaw").largest, 1e-9)

    def test_beta_is_left_out_at_low_speed(self):
        times = np.arange(10) * .001
        speed = np.ones(10)
        speed[3] = .1
        beta_b = np.zeros(10)
        beta_b[3] = 1.
        a = run.from_frame(frame(times, yaw=np.zeros(10), speed=speed, beta=np.zeros(10)), kind="bicycle")
        b = run.from_frame(frame(times, yaw=np.zeros(10), speed=np.ones(10), beta=beta_b.copy()), kind="bicycle")
        self.assertFalse(next(c for c in compare.diff(a, b).columns if c.name == "beta").beyond)
        beta_b[5] = 1.
        b = run.from_frame(frame(times, yaw=np.zeros(10), speed=np.ones(10), beta=beta_b), kind="bicycle")
        beta = next(c for c in compare.diff(a, b).columns if c.name == "beta")
        self.assertEqual((beta.beyond, beta.t_first, beta.largest), (True, .005, 1.))

    def test_not_a_number_in_one_run_is_a_difference(self):
        times = np.arange(10) * .001
        hole = np.zeros(10)
        hole[4:6] = np.nan
        a = run.from_frame(frame(times, value=np.zeros(10)), kind="bicycle")
        b = run.from_frame(frame(times, value=hole), kind="bicycle")
        value = compare.diff(a, b, [("*", compare.Tolerance(1e9))]).columns[0]
        self.assertEqual((value.beyond, value.t_first, value.largest), (True, .004, np.inf))
        self.assertTrue(compare.diff(b, a).differs())
        self.assertFalse(compare.diff(b, b).differs())
        infinite = run.from_frame(frame(times, value=np.full(10, np.inf)), kind="bicycle")
        self.assertTrue(compare.diff(a, infinite).differs())
        self.assertFalse(compare.diff(infinite, infinite).differs())

    def test_cli_and_no_matplotlib(self):
        with tempfile.TemporaryDirectory() as directory:
            path_a = Path(directory) / "a.csv"
            path_b = Path(directory) / "b.csv"
            times = np.arange(1000) * .001
            data = frame(times, yaw=np.zeros(1000), speed=np.ones(1000), value=np.zeros(1000))
            write_run(path_a, data)
            changed = data.copy()
            changed.loc[250:, "value"] = 1e-6
            write_run(path_b, changed)
            self.assertEqual(self._cli([str(path_a), str(path_a)]), 0)
            self.assertEqual(self._cli([str(path_a), str(path_b)]), 1)
            self.assertEqual(self._cli([str(path_a), str(path_b), "--tol", "*=1e-5"]), 0)
            self.assertEqual(self._cli([str(path_a), str(path_a) + ".missing"]), 2)
            root = Path(__file__).resolve().parents[2]
            code = ("import sys; from rallytools import cli; cli.main(['diff', sys.argv[1], sys.argv[1]]); "
                    "print('matplotlib' in sys.modules)")
            output = subprocess.run([sys.executable, "-c", code, str(path_a)],
                                    cwd=root / "tools", capture_output=True, text=True, check=True)
            self.assertEqual(output.stdout.splitlines()[-1], "False")

    def test_cli_all_order(self):
        # Beyond: x first at 0.5 s (larger), y first at 0.1 s (smaller). Within: w larger than z.
        times = np.arange(1000) * .001
        data = frame(times, x=np.zeros(1000), y=np.zeros(1000), z=np.zeros(1000), w=np.zeros(1000),
                     yaw=np.zeros(1000), speed=np.ones(1000))
        changed = data.copy()
        changed.loc[500:, "x"] = 1e-6
        changed.loc[100:, "y"] = 1e-7
        changed["z"] = 1e-12
        changed["w"] = 1e-10
        with tempfile.TemporaryDirectory() as directory:
            path_a, path_b = Path(directory) / "a.csv", Path(directory) / "b.csv"
            write_run(path_a, data)
            write_run(path_b, changed)
            from rallytools import cli
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(cli.main(["diff", str(path_a), str(path_b), "--tol", "*=1e-8", "--all"]), 1)
        names = [line.split("\t")[0] for line in output.getvalue().splitlines() if "\t" in line]
        self.assertEqual(names[:5], ["column", "y", "x", "w", "z"])
        self.assertIn("column\tlargest\tt_largest\tt_first", output.getvalue().splitlines())

    def test_cli_reports_metadata_only_differences(self):
        with tempfile.TemporaryDirectory() as directory:
            path_a = Path(directory) / "a.csv"
            path_b = Path(directory) / "b.csv"
            data = frame([0.], yaw=[0.], speed=[1.], value=[0.])
            write_run(path_a, data, meta={"mass": 1.})
            write_run(path_b, data, meta={"gravity": 9.81})
            from rallytools import cli
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                result = cli.main(["diff", str(path_a), str(path_b)])
            self.assertEqual(result, 0)
            self.assertIn("meta only A: mass", output.getvalue())
            self.assertIn("meta only B: gravity", output.getvalue())

    def test_cli_line_1_values_and_missing_columns(self):
        with tempfile.TemporaryDirectory() as directory:
            path_a, path_b = Path(directory) / "a.csv", Path(directory) / "b.csv"
            data = frame([0.], yaw=[0.], speed=[1.], value=[0.])
            keys = ("zeta", "mass", "alpha", "lf", "spring_front", "bump")
            write_run(path_a, data, meta={key: 1. for key in keys})
            write_run(path_b, data, meta={key: 2. for key in keys})
            from rallytools import cli
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                self.assertEqual(cli.main(["diff", str(path_a), str(path_b)]), 0)
            # A changed setup is printed, in the order of line 1, and is not a difference.
            self.assertEqual([line for line in output.getvalue().splitlines() if line.startswith("meta")],
                             [f"meta {key}: 1.0 -> 2.0" for key in keys])
            write_run(path_b, data.assign(extra=[1.]))
            self.assertEqual(self._cli([str(path_a), str(path_b)]), 1)
            self.assertEqual(self._cli([str(path_a), str(path_b), "--ignore-missing"]), 0)
            (Path(directory) / "empty.csv").write_text("step,time,yaw,speed\n")
            self.assertEqual(self._cli([str(path_a), str(Path(directory) / "empty.csv")]), 2)

    @staticmethod
    def _cli(args):
        from rallytools import cli
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            return cli.main(["diff", *args])


if __name__ == "__main__":
    unittest.main()
