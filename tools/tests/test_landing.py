from pathlib import Path
import subprocess
import sys
import unittest

import numpy as np

from rallytools import cli
from rallytools.landing import text
from rallytools.run import load
from .test_events import four, flight

FIXTURES = Path(__file__).parent / "fixtures"
TOOLS = Path(__file__).resolve().parents[1]


def landing_run(meta):
    fz = flight()
    fz[450] = 19500.0
    defl = np.where(np.arange(1000) == 500, 0.15, 0.05)
    return four(fz, meta=meta, **{"wheel.FL.deflection": defl})


class TestText(unittest.TestCase):
    def test_numbers_and_window(self):
        meta = {"rest_deflection_front": 0.048657599999999995, "bump_travel_front": 0.25,
                "vertical": 2, "fz0_front": 3000.0}
        out = text(landing_run(meta))
        for part in ("101.3 mm", "148.7 mm", "6.50 x fz0", "window 2.0 s", "touchdown 0.3 s after 0.3 s"):
            self.assertIn(part, out)

    def test_stop_and_settling_lines(self):
        # 37 rows of bump-stop force at 0.5 ms: 18.5 ms; pos.z dips and rises back once, 10% of the dip.
        stop = np.zeros(1000)
        stop[400:437] = 5000.0
        z = np.full(1000, 0.5)
        z[400] = 0.4
        z[500] = 0.51
        run = four(flight(), run_dt=0.0005, **{"wheel.FL.stopF": stop, "pos.z": z})
        out = text(run)
        self.assertIn("stop 5000 N for 18.5 ms", out)
        self.assertIn("settling 0.000 s; rebounds 1, overshoot 10.0 %", out)

    def test_each_wheel_has_its_own_line(self):
        fz = np.column_stack([flight()] * 4)
        fz[450] = (6000.0, 7000.0, 8000.0, 9000.0)
        out = text(four(fz))
        for part in ("FL  peak Fz 6000 N", "FR  peak Fz 7000 N", "RL  peak Fz 8000 N", "RR  peak Fz 9000 N"):
            self.assertIn(part, out)

    def test_no_landing(self):
        self.assertIn("no landing in this run", text(load(FIXTURES / "four_wheel_unsprung.csv")))

    def test_missing_rest_key_is_not_defaulted(self):
        out = text(landing_run({"vertical": 2, "fz0_front": 3000.0}))
        self.assertIn("rerun with a newer build", out)
        fl = [line for line in out.splitlines() if line.strip().startswith("FL")][0]
        self.assertNotIn(" mm", fl.split("travel")[1].split("margin")[0].replace(
            "n/a: no rest_deflection on line 1, rerun with a newer build", ""))


class TestCli(unittest.TestCase):
    def test_missing_file(self):
        self.assertEqual(cli.main(["landing", str(FIXTURES / "nope.csv")]), 2)

    def test_bad_window(self):
        self.assertEqual(cli.main(["landing", str(FIXTURES / "four_wheel_unsprung.csv"), "--window", "0"]), 2)

    def test_good_run(self):
        self.assertEqual(cli.main(["landing", str(FIXTURES / "four_wheel_unsprung.csv")]), 0)

    def test_no_matplotlib(self):
        code = ("import sys; from rallytools import cli; "
                f"r = cli.main(['landing', r'{FIXTURES / 'four_wheel_unsprung.csv'}']); "
                "sys.exit(r or (3 if 'matplotlib' in sys.modules else 0))")
        result = subprocess.run([sys.executable, "-c", code], cwd=TOOLS, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
