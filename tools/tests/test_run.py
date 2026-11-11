import io
import os
import subprocess
import sys
import unittest

from rallytools import run as R


def load(text, name="t"):
    return R.load(io.StringIO(text), name=name)


def four(meta=""):
    return load(f"# schema=6 dt=0.001 {meta} units: step[count]\nstep,time,wheel.FL.slipRatio\n0,0,0\n")


class TestRun(unittest.TestCase):
    def test_line1(self):
        r = load("# schema=6 dt=0.0005 model=4 lf=1.3 units: step[count] time[s] torque.z[N m]\n"
                 "step,time,yaw,speed,torque.z\n0,0,0,1,0\n1,0.1,0,1,0\n")
        self.assertEqual(r.schema, 6)
        self.assertIsInstance(r.schema, int)
        self.assertEqual(r.dt, 0.0005)
        self.assertEqual(r.meta, {"model": 4.0, "lf": 1.3})
        self.assertEqual(r.units["torque.z"], "N m")

    def test_single_track_with_wheel_fz(self):
        r = load("step,time,pos.x,pos.y,yaw,speed,steer,axle.front.Fy,wheel.FL.Fz\n0,0,0,0,0,1,0,0,0\n")
        self.assertEqual(r.kind, "single_track")

    def test_kinds(self):
        for header, kind in (("quat.w", "rigid_body"), ("wheel.FL.slipRatio", "four_wheel"),
                             ("axle.front.Fy", "single_track"), ("yaw,speed", "bicycle")):
            r = load(f"step,time,{header}\n" + "0," * (2 + header.count(",")) + "0\n")
            self.assertEqual(r.kind, kind)
        with self.assertRaises(ValueError):
            load("step,time,foo\n0,0,0\n")

    def test_no_comment_line(self):
        r = load("step,time,yaw,speed\n0,0,0,1\n")
        self.assertIsNone(r.schema)
        self.assertIsNone(r.dt)
        self.assertEqual(r.meta, {})

    def test_vertical(self):
        self.assertEqual(four("vertical=2").vertical, "unsprung")
        self.assertEqual(four("springs=1").vertical, "springs")
        self.assertEqual(four("springs=0").vertical, "planar")
        self.assertIsNone(four("").vertical)

    def test_column_error(self):
        r = four()
        with self.assertRaises(ValueError) as cm:
            r.column("nope")
        self.assertIn("nope", str(cm.exception))
        self.assertIn("t", str(cm.exception))
        self.assertTrue(r.has("step"))

    def test_no_matplotlib(self):
        out = subprocess.run([sys.executable, "-c", "import rallytools.run, sys; print('matplotlib' in sys.modules)"],
                             capture_output=True, text=True, check=True,
                             cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))).stdout.strip()
        self.assertEqual(out, "False")


if __name__ == "__main__":
    unittest.main()
