import unittest

import numpy as np
import pandas as pd

from rallytools.events import Event, airborne, events, landings, rebounds, settling_time
from rallytools.run import from_frame

WHEELS = ("FL", "FR", "RL", "RR")


def four(fz=None, rows=1000, dt=0.001, meta=None, run_dt=None, **columns):
    """A four-wheel run: fz is an array (rows,) for all wheels or (rows, 4); other columns are constant."""
    t = np.arange(rows) * dt
    fz = np.full(rows, 3000.0) if fz is None else np.asarray(fz, dtype=float)
    if fz.ndim == 1:
        fz = np.column_stack([fz] * 4)
    data = {"time": t, "pos.z": np.full(rows, 0.5), "speed": np.full(rows, 20.0)}
    for i, w in enumerate(WHEELS):
        data[f"wheel.{w}.Fz"] = fz[:, i]
        data[f"wheel.{w}.deflection"] = np.full(rows, 0.05)
        data[f"wheel.{w}.slipRatio"] = np.zeros(rows)
        data[f"wheel.{w}.omega"] = np.full(rows, 66.7)
        data[f"wheel.{w}.stopF"] = np.zeros(rows)
    for name, values in columns.items():
        data[name] = values
    return from_frame(pd.DataFrame(data), kind="four_wheel", dt=run_dt,
                      meta={"wheel_radius": 0.3, **(meta or {})})


def flight(rows=1000, dt=0.001, gaps=((0.0, 0.3),)):
    t = np.arange(rows) * dt
    fz = np.full(rows, 3000.0)
    for a, b in gaps:
        fz[(t >= a) & (t < b)] = 0.0
    return fz


class TestAirborne(unittest.TestCase):
    def test_all_four_needed(self):
        fz = np.column_stack([flight()] * 4)
        t = np.arange(1000) * 0.001
        got = airborne(t, fz)
        self.assertEqual(len(got), 1)
        self.assertAlmostEqual(got[0][0], 0.0, delta=1e-12)
        self.assertAlmostEqual(got[0][1], 0.3, delta=1e-12)
        fz[:, 0] = 3000.0
        self.assertEqual(airborne(t, fz), [])

    def test_two_flights_two_landings(self):
        fz = flight(gaps=((0.0, 0.3), (0.6, 0.7)))
        t = np.arange(1000) * 0.001
        got = airborne(t, np.column_stack([fz] * 4))
        self.assertEqual(len(got), 2)
        self.assertAlmostEqual(got[1][1], 0.7, delta=1e-12)
        found = landings(four(fz))
        self.assertEqual(len(found), 2)
        self.assertAlmostEqual(found[0].t_touchdown, 0.3, delta=1e-12)
        self.assertAlmostEqual(found[1].t_touchdown, 0.7, delta=1e-12)
        # The first landing stops before the second flight (its last row is 0.599): a heavier second landing
        # must not show in the first one's peak.
        heavier = fz.copy()
        heavier[np.arange(1000) * 0.001 >= 0.7] = 5000.0
        first, second = landings(four(heavier))
        self.assertEqual(first.peak_fz["FL"], 3000.0)
        self.assertEqual(second.peak_fz["FL"], 5000.0)

    def test_run_ending_airborne_has_no_landing(self):
        fz = flight(gaps=((0.5, 2.0),))
        self.assertEqual(landings(four(fz)), [])
        t = np.arange(1000) * 0.001
        self.assertAlmostEqual(airborne(t, np.column_stack([fz] * 4))[0][1], t[-1], delta=1e-12)


class TestEvents(unittest.TestCase):
    def test_lock_threshold_and_guards(self):
        slip = np.zeros(1000)
        slip[200:500] = -1.0
        self.assertEqual(events(four(**{"wheel.FL.slipRatio": slip})),
                         [Event("lock", "FL", 0.2, 0.5)])
        slip[200:500] = -0.94
        self.assertEqual(events(four(**{"wheel.FL.slipRatio": slip})), [])
        self.assertEqual(events(four(**{"speed": np.full(1000, 0.9),
                                        "wheel.FL.slipRatio": np.full(1000, -1.0)})), [])
        fz = np.full((1000, 4), 3000.0)
        fz[200:500, 0] = 0.0
        lift_slip = np.zeros(1000)
        lift_slip[200:500] = -1.0
        self.assertEqual(events(four(fz, **{"wheel.FL.slipRatio": lift_slip})),
                         [Event("lift", "FL", 0.2, 0.5)])

    def test_spin_threshold_and_radius(self):
        slip = np.zeros(1000)
        slip[300:400] = 0.5
        self.assertEqual(events(four(**{"wheel.RR.slipRatio": slip})),
                         [Event("spin", "RR", 0.3, 0.4)])
        slip[300:400] = 0.19
        self.assertEqual(events(four(**{"wheel.RR.slipRatio": slip})), [])
        self.assertEqual(events(four(**{"wheel.RR.slipRatio": np.where(
            np.arange(1000) >= 300, 0.5, 0.0), "wheel.RR.omega": np.full(1000, 3.0)})), [])
        fz = np.full((1000, 4), 3000.0)
        fz[300:400, 3] = 0.0
        slip[300:400] = 0.5
        self.assertEqual(events(four(fz, **{"wheel.RR.slipRatio": slip})),
                         [Event("lift", "RR", 0.3, 0.4)])
        run = four()
        run = from_frame(run.data, kind="four_wheel", meta={})
        with self.assertRaisesRegex(ValueError, "wheel_radius"):
            events(run)

    def test_minimum_duration_and_stops(self):
        slip = np.zeros(1000)
        slip[200:249] = -1.0
        slip[300:350] = -1.0
        found = events(four(**{"wheel.FL.slipRatio": slip}))
        self.assertEqual(found, [Event("lock", "FL", 0.3, 0.35000000000000003)])
        self.assertEqual(events(four(**{"wheel.FL.slipRatio": slip}), min_time=0),
                         [Event("lock", "FL", 0.2, 0.249),
                          Event("lock", "FL", 0.3, 0.35000000000000003)])
        stop = np.zeros(1000)
        stop[400:403] = 5000.0
        stop[500:502] = -100.0
        self.assertEqual(events(four(**{"wheel.FL.stopF": stop})),
                         [Event("bump_stop", "FL", 0.4, 0.403),
                          Event("droop_stop", "FL", 0.5, 0.502)])
        data = four()
        data = from_frame(data.data.drop(columns="wheel.FL.stopF"), kind="four_wheel",
                          meta=data.meta)
        self.assertEqual(events(data), [])

    def test_order_and_end_time(self):
        lock = np.zeros(1000)
        lock[100:] = -1.0
        spin = np.zeros(1000)
        spin[100:] = 0.5
        fz = np.full((1000, 4), 3000.0)
        fz[50:100, 1] = 0.0
        found = events(four(fz, **{"wheel.RL.slipRatio": lock,
                                   "wheel.FL.slipRatio": spin}))
        self.assertEqual([(event.kind, event.wheel) for event in found[:3]],
                         [("lift", "FR"), ("spin", "FL"), ("lock", "RL")])
        self.assertEqual(events(four(rows=300, **{"wheel.FL.slipRatio": np.r_[
            np.zeros(200), np.full(100, -1.0)]})),
                         [Event("lock", "FL", 0.2, 0.299)])
        bicycle = from_frame(pd.DataFrame({"time": [0.0], "yaw": [0.0], "speed": [1.0]}),
                             kind="bicycle", meta={"wheel_radius": 0.3})
        with self.assertRaises(ValueError):
            events(bicycle)


class TestWindow(unittest.TestCase):
    def late_peak(self, **kw):
        fz = flight(**kw)
        fz[np.arange(1000) * 0.001 >= 0.45] = 5000.0
        return fz

    def test_window_limits_the_rows(self):
        run = four(self.late_peak())
        self.assertEqual(landings(run, 0.1)[0].peak_fz["FL"], 3000.0)
        self.assertEqual(landings(run)[0].peak_fz["FL"], 5000.0)

    def test_window_end_is_inclusive(self):
        fz = flight()
        fz[400] = 5000.0
        run = four(fz)
        self.assertEqual(landings(run, 0.1)[0].peak_fz["FL"], 5000.0)
        self.assertEqual(landings(run, 0.099)[0].peak_fz["FL"], 3000.0)

    def test_window_stops_before_next_flight(self):
        fz = flight(gaps=((0.0, 0.3), (0.6, 0.7)))
        fz[np.arange(1000) * 0.001 >= 0.7] = 5000.0
        self.assertEqual(landings(four(fz), 5.0)[0].peak_fz["FL"], 3000.0)

    def test_window_must_be_positive(self):
        run = four(flight())
        for window in (0, -1):
            with self.assertRaises(ValueError):
                landings(run, window)


class TestSettling(unittest.TestCase):
    def setUp(self):
        self.t = np.arange(3001) * 0.001
        self.fz = np.column_stack([3000.0 * (1 + 0.5 * np.exp(-self.t / 0.2))] * 4)

    def test_last_exit_from_band(self):
        self.assertAlmostEqual(settling_time(self.t, self.fz), 0.461, delta=1e-9)
        fz = self.fz.copy()
        fz[(self.t >= 1.0) & (self.t < 1.1), 3] += 300.0
        self.assertAlmostEqual(settling_time(self.t, fz), 1.1, delta=1e-9)

    def test_time_is_counted_from_the_first_row(self):
        self.assertAlmostEqual(settling_time(self.t + 5.0, self.fz), 0.461, delta=1e-9)

    def test_landing_settles_from_touchdown(self):
        # Touchdown at 0.3 s; the loads decay into the 5% band 0.05 ln 10 = 0.1151 s later: the row at 0.416.
        t = np.arange(1000) * 0.001
        fz = np.where(t < 0.3, 0.0, 3000.0 * (1 + 0.5 * np.exp(-(t - 0.3) / 0.05)))
        self.assertAlmostEqual(landings(four(fz))[0].settling_time, 0.116, delta=1e-9)

    def test_still_moving_is_none(self):
        t = np.arange(3001) * 0.001
        fz = np.full(3001, 3000.0)
        ramp = t >= 2.5
        fz[ramp] = 3000.0 * (1 + 0.02 * (t[ramp] - 2.5) / 0.5)
        self.assertIsNone(settling_time(t, np.column_stack([fz] * 4)))


class TestRebounds(unittest.TestCase):
    def test_damped_oscillation(self):
        t = np.arange(6001) * 0.001
        z = 0.5 - 0.08 * np.exp(-2 * t) * np.cos(10 * t)
        count, overshoot = rebounds(z)
        self.assertEqual(count, 3)
        self.assertAlmostEqual(overshoot, 0.5442, delta=1e-3)

    def test_only_after_the_lowest_point(self):
        # A bump on the way down is not a rebound: one maximum after the dip, 10% of it.
        z = np.full(100, 0.5)
        z[10], z[20], z[30] = 0.52, 0.4, 0.51
        count, overshoot = rebounds(z)
        self.assertEqual(count, 1)
        self.assertAlmostEqual(overshoot, 0.1, delta=1e-12)


class TestMetrics(unittest.TestCase):
    def landing_run(self, meta=None, **columns):
        fz = flight()
        defl = np.where(np.arange(1000) == 500, 0.15, 0.05)
        return four(fz, meta=meta, **{"wheel.FL.deflection": defl}, **columns)

    def test_travel_and_margin(self):
        meta = {"rest_deflection_front": 0.048657599999999995, "bump_travel_front": 0.25, "vertical": 2}
        land = landings(self.landing_run(meta))[0]
        self.assertAlmostEqual(land.peak_travel["FL"], 0.1013424, delta=1e-12)
        self.assertAlmostEqual(land.travel_margin["FL"], 0.1486576, delta=1e-12)
        land = landings(self.landing_run({k: v for k, v in meta.items() if k != "rest_deflection_front"}))[0]
        self.assertIsNone(land.peak_travel["FL"])
        self.assertIsNone(land.travel_margin["FL"])
        land = landings(self.landing_run({**meta, "vertical": 1}))[0]
        self.assertAlmostEqual(land.peak_travel["FL"], 0.1013424, delta=1e-12)
        self.assertIsNone(land.travel_margin["FL"])

    def test_stop_time_counts_positive_rows(self):
        stop = np.zeros(1000)
        stop[400:437] = 5000.0
        stop[500:520] = -100.0
        land = landings(four(flight(), run_dt=0.0005, **{"wheel.FL.stopF": stop}))[0]
        self.assertAlmostEqual(land.stop_time["FL"], 0.0185, delta=1e-12)
        self.assertEqual(land.peak_stop["FL"], 5000.0)
        self.assertEqual(land.peak_stop["FR"], 0.0)
        self.assertIsNone(landings(four(flight(), **{"wheel.FL.stopF": stop}))[0].stop_time["FL"])

    def test_metrics_cover_the_landing_rows_only(self):
        # Larger values while still in the air (row 100) belong to no landing.
        defl = np.full(1000, 0.05)
        defl[100], defl[500] = 0.2, 0.15
        stop = np.zeros(1000)
        stop[100:110] = 9000.0
        stop[400:437] = 5000.0
        run = four(flight(), run_dt=0.0005, meta={"rest_deflection_front": 0.05},
                   **{"wheel.FL.deflection": defl, "wheel.FL.stopF": stop})
        land = landings(run)[0]
        self.assertAlmostEqual(land.peak_travel["FL"], 0.1, delta=1e-12)
        self.assertEqual(land.peak_stop["FL"], 5000.0)
        self.assertAlmostEqual(land.stop_time["FL"], 0.0185, delta=1e-12)

    def test_peak_fz_nominal(self):
        fz = flight()
        fz[450] = 19500.0
        land = landings(four(fz, meta={"fz0_front": 3000.0}))[0]
        self.assertEqual(land.peak_fz["FL"], 19500.0)
        self.assertEqual(land.peak_fz_nominal["FL"], 6.5)
        self.assertIsNone(landings(four(fz))[0].peak_fz_nominal["FL"])

    def test_rear_wheels_use_rear_keys(self):
        fz = flight()
        fz[450] = 19500.0
        meta = {"fz0_front": 3000.0, "fz0_rear": 3900.0,
                "rest_deflection_front": 0.048657599999999995, "rest_deflection_rear": 0.0533664}
        land = landings(four(fz, meta=meta))[0]
        self.assertEqual(land.peak_fz_nominal["RL"], 5.0)
        self.assertEqual(land.peak_fz_nominal["RR"], 5.0)
        # deflection is 0.05 everywhere in this run
        self.assertAlmostEqual(land.peak_travel["RL"], 0.05 - 0.0533664, delta=1e-12)
        self.assertAlmostEqual(land.peak_travel["FR"], 0.05 - 0.048657599999999995, delta=1e-12)

    def test_no_flight_no_landing(self):
        self.assertEqual(landings(four()), [])

    def test_other_kinds_rejected(self):
        run = from_frame(pd.DataFrame({"time": [0.0], "yaw": [0.0], "speed": [1.0]}), kind="bicycle")
        with self.assertRaises(ValueError):
            landings(run)


if __name__ == "__main__":
    unittest.main()
