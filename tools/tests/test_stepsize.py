import contextlib
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import pandas as pd

from rallytools import cli, run, stepsize
from rallytools.compare import Tolerance
from . import support
from .support import write_run

DTS = (0.004, 0.002, 0.001)


def make(dt, **functions):
    t = np.arange(round(1 / dt) + 1) * dt
    data = {"step": np.arange(len(t)), "time": t, "yaw": np.zeros(len(t)), "speed": np.ones(len(t))}
    for name, f in functions.items():
        data[name] = f(t, dt)
    return pd.DataFrame(data)


EXE = os.environ.get("RALLYSIM_EXE")


def frames(**functions):
    return [make(dt, **functions) for dt in DTS]


def runs(units=None, **functions):
    return [run.from_frame(frame, kind="bicycle", dt=dt, units=units) for frame, dt in zip(frames(**functions), DTS)]


def row(rows, name):
    return next(r for r in rows if r.name == name)


def write_all(folder, data, prefix, units=None):
    paths = []
    for name, dt, frame in zip("abc", DTS, data):
        path = Path(folder) / f"{prefix}{name}.csv"
        write_run(path, frame, dt=dt, units=units)
        paths.append(str(path))
    return paths


class TestStepsize(unittest.TestCase):
    def test_first_and_second_order(self):
        rows = stepsize.report(*runs(p=lambda t, dt: t ** 2 + dt, q=lambda t, dt: t + dt ** 2))
        p, q = row(rows, "p"), row(rows, "q")
        self.assertAlmostEqual(p.d1, 0.002, delta=1e-12)
        self.assertAlmostEqual(p.d2, 0.001, delta=1e-12)
        self.assertAlmostEqual(p.ratio, 0.5, delta=1e-9)
        self.assertEqual(p.verdict, "converging")
        self.assertAlmostEqual(q.ratio, 0.25, delta=1e-9)
        self.assertEqual(q.verdict, "converging")

    def test_offset_equal_and_text(self):
        a, b, c = runs(r=lambda t, dt: t, s=lambda t, dt: t)
        b.data["r"] = b.data["r"] + 1e-3
        rows = stepsize.report(a, b, c)
        r = row(rows, "r")
        self.assertAlmostEqual(r.d1, 1e-3, delta=1e-12)
        self.assertAlmostEqual(r.d2, 1e-3, delta=1e-12)
        self.assertAlmostEqual(r.ratio, 1.0, delta=1e-9)
        self.assertEqual(r.verdict, "not converging")
        s = row(rows, "s")
        self.assertEqual((s.verdict, s.ratio), ("equal", None))
        lines = stepsize.text(rows).splitlines()
        self.assertEqual(lines[0], "column\td1\td2\tratio\tverdict")
        self.assertEqual([line.split("\t")[0] for line in lines[1:]], ["r"])
        names = [line.split("\t")[0] for line in stepsize.text(rows, True).splitlines()[1:]]
        self.assertEqual(names[0], "r")
        self.assertIn("s", names)

    def test_change_at_the_finest_step_and_order(self):
        a, b, c = runs(u=lambda t, dt: t, r=lambda t, dt: t, w=lambda t, dt: t)
        c.data["u"] = c.data["u"] + 5e-4
        b.data["r"] = b.data["r"] + 1e-3
        b.data["w"] = b.data["w"] + 2e-3
        rows = stepsize.report(a, b, c)
        u = row(rows, "u")
        self.assertEqual((u.d1, u.ratio, u.verdict), (0.0, None, "not converging"))
        names = [line.split("\t")[0] for line in stepsize.text(rows).splitlines()[1:]]
        self.assertEqual(names, ["w", "r", "u"])

    def test_nan_is_not_converging(self):
        a, b, c = runs(n=lambda t, dt: t)
        b.data.loc[10, "n"] = np.nan
        self.assertEqual(row(stepsize.report(a, b, c), "n").verdict, "not converging")

    def test_bad_steps(self):
        a, b, c = runs(p=lambda t, dt: t)
        wrong = run.from_frame(make(0.002, p=lambda t, dt: t), kind="bicycle", dt=0.002)
        with self.assertRaises(ValueError):
            stepsize.report(a, b, wrong)
        nodt = run.from_frame(make(0.001, p=lambda t, dt: t), kind="bicycle")
        with self.assertRaises(ValueError):
            stepsize.report(a, b, nodt)

    def test_cli(self):
        with tempfile.TemporaryDirectory() as folder:
            good = write_all(folder, frames(p=lambda t, dt: t ** 2 + dt, q=lambda t, dt: t + dt ** 2,
                                            s=lambda t, dt: t), "g")
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(["stepsize", *good]), 0)
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli.main(["stepsize", good[0], good[1], str(Path(folder) / "none.csv")]), 2)
            data = frames(r=lambda t, dt: t)
            data[1]["r"] += 1e-3
            bad = write_all(folder, data, "x")
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                self.assertEqual(cli.main(["stepsize", *bad]), 1)
            lines = out.getvalue().splitlines()
            self.assertEqual(lines[3], "column\td1\td2\tratio\tverdict")
            self.assertTrue(lines[4].startswith("r\t"))

    def test_negligible_excuses_only_what_does_not_converge(self):
        # unit "N m": a column of size 100 that converges (first order, differences about 2e-6 and 1e-6, well below
        # 1e-6 x 100), and a small one with a difference of 1e-5 that does not shrink
        a, b, c = runs(units={"big": "N m", "small": "N m"}, big=lambda t, dt: 100 + 1e-3 * dt, small=lambda t, dt: 0 * t)
        b.data["small"] = b.data["small"] + 1e-5
        rows = stepsize.report(a, b, c)
        big, small = row(rows, "big"), row(rows, "small")
        self.assertEqual(big.verdict, "converging")  # the tolerance does not turn it into "equal"
        self.assertAlmostEqual(big.ratio, 0.5, delta=1e-6)
        self.assertEqual(small.verdict, "negligible")
        self.assertAlmostEqual(small.ratio, 1.0, delta=1e-9)
        # the strict rule: the same column does not converge
        strict = stepsize.report(a, b, c, rel_tol=0)
        self.assertEqual(row(strict, "small").verdict, "not converging")
        self.assertEqual(row(strict, "big").verdict, "converging")

    def test_cli_negligible_does_not_fail(self):
        with tempfile.TemporaryDirectory() as folder:
            data = frames(big=lambda t, dt: 100 + 1e-3 * dt, small=lambda t, dt: 0 * t)
            data[1]["small"] += 1e-5
            paths = write_all(folder, data, "u", units={"big": "N m", "small": "N m"})
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                self.assertEqual(cli.main(["stepsize", *paths]), 0)
                self.assertEqual(cli.main(["stepsize", *paths, "--all"]), 0)
            self.assertRegex(out.getvalue(), r"(?m)^small\t.*\tnegligible\s*$")
            self.assertRegex(out.getvalue(), r"(?m)^big\t.*\tconverging\s*$")
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(["stepsize", *paths, "--rel", "0"]), 1)
                self.assertEqual(cli.main(["stepsize", *paths, "--rel", "1e-9"]), 1)

    def test_converging_below_the_unit_tolerance_stays_converging(self):
        a, b, c = runs(units={"small": "m", "big": "m"}, small=lambda t, dt: t + 1e-3 * dt, big=lambda t, dt: 1e4 + 0 * t)
        small = row(stepsize.report(a, b, c), "small")
        self.assertEqual(small.verdict, "converging")
        self.assertAlmostEqual(small.ratio, 0.5, delta=1e-6)

    def test_scale_is_the_largest_over_the_unit_and_the_runs(self):
        # the large column is only large in the finest run, and a column of another unit does not count
        a, b, c = runs(units={"big": "N m", "small": "N m", "other": "N"}, big=lambda t, dt: 0 * t,
                       small=lambda t, dt: 0 * t, other=lambda t, dt: 1e6 + 0 * t)
        c.data["big"] = c.data["big"] + 100
        b.data["small"] = b.data["small"] + 1e-5
        self.assertEqual(row(stepsize.report(a, b, c), "small").verdict, "negligible")
        # next to a column of another unit only, it is measured against its own unit
        a, b, c = runs(units={"small": "N m", "other": "N"}, small=lambda t, dt: 0 * t, other=lambda t, dt: 1e6 + 0 * t)
        b.data["small"] = b.data["small"] + 1e-5
        self.assertEqual(row(stepsize.report(a, b, c), "small").verdict, "not converging")

    def test_column_without_a_unit_uses_its_own_size(self):
        a, b, c = runs(own=lambda t, dt: 100 + 0 * t, small=lambda t, dt: 0 * t, other=lambda t, dt: 100 + 0 * t)
        b.data["own"] = b.data["own"] + 1e-5
        b.data["small"] = b.data["small"] + 1e-5
        rows = stepsize.report(a, b, c)
        self.assertEqual(row(rows, "own").verdict, "negligible")  # 1e-5 < 1e-6 x its own 100
        # no unit means no sharing: the small column is not measured against the others' size
        self.assertEqual(row(rows, "small").verdict, "not converging")

    def test_absolute_floor_gives_equal(self):
        # a column of size 1e-6 in a unit of its own: a difference of 1e-10 is under the 1e-9 floor, so "equal"
        a, b, c = runs(units={"tiny": "m"}, tiny=lambda t, dt: 1e-6 + 0 * t)
        b.data["tiny"] = b.data["tiny"] + 1e-10
        self.assertEqual(row(stepsize.report(a, b, c), "tiny").verdict, "equal")
        self.assertEqual(row(stepsize.report(a, b, c, abs_tol=0), "tiny").verdict, "not converging")

    def test_fresh_runs(self):
        support.need(self, EXE and os.path.isfile(EXE), "RALLYSIM_EXE must name an existing rallysim executable")
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as folder:
            def at(scenario, name, *settings):
                path = Path(folder) / f"{scenario}_{name}.csv"
                result = subprocess.run([EXE, str(root / "scenarios" / f"{scenario}.ini"), str(path), *settings],
                                        capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                return str(path)
            lane = [at("fw_lane", "a"), at("fw_lane", "b", "--set", "run.dt=0.5ms"),
                    at("fw_lane", "c", "--set", "run.dt=0.25ms")]
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(cli.main(["stepsize", *lane, "--all"]), 0, out.getvalue())
            text = out.getvalue()
            for corner in ("FL", "FR", "RL", "RR"):
                self.assertRegex(text, rf"(?m)^wheel\.{corner}\.brakeTorque\t.*\tnegligible\s*$")
            self.assertRegex(text, r"(?m)^roll\t.*\tconverging\s*$")
            self.assertRegex(text, r"(?m)^wheel\.FL\.deflection\t.*\tconverging\s*$")
            # the strict rule shows the blip as not converging
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(["stepsize", *lane, "--rel", "0"]), 1)
            # a rule of 0 for the blip turns the automatic excuse off for it alone
            with contextlib.redirect_stdout(io.StringIO()) as out:
                self.assertEqual(cli.main(["stepsize", *lane, "--tol", "wheel.*.brakeTorque=0"]), 1)
            self.assertRegex(out.getvalue(), r"(?m)^wheel\.FL\.brakeTorque\t.*\tnot converging\s*$")

    def blip(self):
        """Runs of a column "small" that does not shrink (1e-5 between the first two runs, nothing after) and a
        column "big" that converges, both in N m."""
        a, b, c = runs(units={"big": "N m", "small": "N m"}, big=lambda t, dt: 100 + 1e-3 * dt, small=lambda t, dt: 0 * t)
        b.data["small"] = b.data["small"] + 1e-5
        return a, b, c

    def test_a_rule_larger_than_the_differences_excuses_a_column(self):
        a, b, c = runs(units={"n": "m"}, n=lambda t, dt: 100 + 0 * t)
        b.data["n"] = b.data["n"] + 5e-4
        self.assertEqual(row(stepsize.report(a, b, c), "n").verdict, "not converging")
        # the limit is the rule's, not the unit's: 1e-2 absolute excuses it, 1e-4 does not
        self.assertEqual(row(stepsize.report(a, b, c, rules=[("n", Tolerance(1e-2, 0))]), "n").verdict, "negligible")
        self.assertEqual(row(stepsize.report(a, b, c, rules=[("n", Tolerance(1e-4, 0))]), "n").verdict,
                         "not converging")
        # REL is a share of the column's largest value over the runs (about 100): 1e-5 x 100 = 1e-3
        self.assertEqual(row(stepsize.report(a, b, c, rules=[("n", Tolerance(0, 1e-5))]), "n").verdict, "negligible")
        self.assertEqual(row(stepsize.report(a, b, c, rules=[("n", Tolerance(0, 1e-6))]), "n").verdict,
                         "not converging")
        # --rel does not touch a ruled column
        self.assertEqual(row(stepsize.report(a, b, c, rel_tol=1.0, rules=[("n", Tolerance(1e-4, 0))]), "n").verdict,
                         "not converging")

    def test_a_rule_of_zero_stops_the_automatic_excuse(self):
        a, b, c = self.blip()
        self.assertEqual(row(stepsize.report(a, b, c), "small").verdict, "negligible")
        strict = stepsize.report(a, b, c, rules=[("small", Tolerance(0, 0))])
        self.assertEqual(row(strict, "small").verdict, "not converging")
        # the other column keeps the automatic rule, and the last matching rule wins
        rules = [("*", Tolerance(0, 0)), ("small", Tolerance(1e-3, 0))]
        rows = stepsize.report(a, b, c, rules=rules)
        self.assertEqual(row(rows, "small").verdict, "negligible")

    def test_a_rule_never_changes_a_converging_or_equal_column(self):
        a, b, c = self.blip()
        for tolerance in (Tolerance(0, 0), Tolerance(1e3, 1.0)):
            rows = stepsize.report(a, b, c, rules=[("*", tolerance)])
            big = row(rows, "big")
            self.assertEqual(big.verdict, "converging")
            self.assertAlmostEqual(big.ratio, 0.5, delta=1e-6)
            self.assertEqual(row(rows, "yaw").verdict, "equal")
            self.assertEqual(row(rows, "speed").verdict, "equal")
        # a loose rule does not excuse a NaN
        b.data.loc[10, "big"] = np.nan
        rows = stepsize.report(a, b, c, rules=[("big", Tolerance(1e9, 1e9))])
        self.assertEqual(row(rows, "big").verdict, "not converging")

    def test_cli_tol(self):
        with tempfile.TemporaryDirectory() as folder:
            data = frames(big=lambda t, dt: 100 + 1e-3 * dt, small=lambda t, dt: 0 * t)
            data[1]["small"] += 1e-5
            paths = write_all(folder, data, "t", units={"big": "N m", "small": "N m"})
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cli.main(["stepsize", *paths]), 0)
                self.assertEqual(cli.main(["stepsize", *paths, "--tol", "small=0"]), 1)
                self.assertEqual(cli.main(["stepsize", *paths, "--tol", "small=0", "--tol", "sm*=1e-3"]), 0)
                self.assertEqual(cli.main(["stepsize", *paths, "--tol", "small=0", "--tol", "big=1"]), 1)
                self.assertEqual(cli.main(["stepsize", *paths, "--tol", "big=0"]), 0)
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli.main(["stepsize", *paths, "--tol", "small"]), 2)

    def test_no_matplotlib(self):
        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as folder:
            paths = write_all(folder, frames(p=lambda t, dt: t ** 2 + dt), "m")
            code = ("import sys; from rallytools import cli; cli.main(['stepsize'] + sys.argv[1:]); "
                    "print('matplotlib' in sys.modules)")
            output = subprocess.run([sys.executable, "-c", code, *paths], cwd=root / "tools",
                                    capture_output=True, text=True, check=True)
            self.assertEqual(output.stdout.splitlines()[-1], "False")


if __name__ == "__main__":
    unittest.main()
