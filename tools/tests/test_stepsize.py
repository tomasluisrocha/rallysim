import contextlib
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import pandas as pd

from rallytools import cli, run, stepsize
from .support import write_run

DTS = (0.004, 0.002, 0.001)


def make(dt, **functions):
    t = np.arange(round(1 / dt) + 1) * dt
    data = {"step": np.arange(len(t)), "time": t, "yaw": np.zeros(len(t)), "speed": np.ones(len(t))}
    for name, f in functions.items():
        data[name] = f(t, dt)
    return pd.DataFrame(data)


def frames(**functions):
    return [make(dt, **functions) for dt in DTS]


def runs(**functions):
    return [run.from_frame(frame, kind="bicycle", dt=dt) for frame, dt in zip(frames(**functions), DTS)]


def row(rows, name):
    return next(r for r in rows if r.name == name)


def write_all(folder, data, prefix):
    paths = []
    for name, dt, frame in zip("abc", DTS, data):
        path = Path(folder) / f"{prefix}{name}.csv"
        write_run(path, frame, dt=dt)
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
