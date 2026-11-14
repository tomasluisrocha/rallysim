"""Understeer gradient of four-wheel telemetry runs."""
from __future__ import annotations

import numpy as np

from .run import Run
from .suspension import window

MIN_SPEED = 1.0
MIN_A_LAT_RANGE = 0.5


def understeer(run: Run, start=None, end=None) -> dict:
    """Extra steer against a_lat over the window (the whole run by default), rows at 1 m/s or faster."""
    if run.kind != "four_wheel":
        raise ValueError(f"run '{run.name}': the understeer gradient needs a four-wheel run, not {run.kind}")
    for key in ("lf", "lr"):
        if key not in run.meta:
            raise ValueError(f"run '{run.name}': no '{key}' on line 1")
    if start is None and end is None:
        selected = np.ones(len(run.data), dtype=bool)
    else:
        time = run.column("time")
        selected = window(run, time[0] if start is None else start, end)
    speed = run.column("speed")
    selected = selected & (speed >= MIN_SPEED)
    if not selected.any():
        raise ValueError(f"run '{run.name}': no rows at {MIN_SPEED:g} m/s or faster in the window")
    wheelbase = run.meta["lf"] + run.meta["lr"]
    a_lat = run.column("a_lat")[selected]
    extra = run.column("steer")[selected] - wheelbase * run.column("yaw_rate")[selected] / speed[selected]
    if np.ptp(a_lat) < MIN_A_LAT_RANGE:
        raise ValueError(f"run '{run.name}': a_lat varies by less than {MIN_A_LAT_RANGE:g} m/s^2 "
                         "over the rows used; a steady circle has no slope")
    gradient, intercept = np.polyfit(a_lat, extra, 1)
    front = (run.column("wheel.FL.slipAngle") + run.column("wheel.FR.slipAngle"))[selected] / 2
    rear = (run.column("wheel.RL.slipAngle") + run.column("wheel.RR.slipAngle"))[selected] / 2
    return {
        "a_lat": a_lat,
        "extra_steer": extra,
        "gradient": float(gradient),
        "gradient_deg": float(np.degrees(gradient)),
        "intercept": float(intercept),
        "front_slip": front,
        "rear_slip": rear,
    }
