"""Suspension-window metrics for four-wheel telemetry runs."""
from __future__ import annotations

import numpy as np

from .channels import derived
from .run import Run, WHEELS


def window(run: Run, start=None, end=None) -> np.ndarray:
    time = run.column("time")
    if start is None:
        start = float(time[-1]) - 1.0
    if end is None:
        end = float(time[-1])
    return (time >= start) & (time <= end)


def steady(run: Run, start=None, end=None) -> dict:
    selected = window(run, start, end)
    roll = run.column("roll")
    a_lat = run.column("a_lat")
    roll_window = roll[selected]
    a_lat_window = a_lat[selected]
    front_window = derived(run, "transfer.front")[selected]
    rear_window = derived(run, "transfer.rear")[selected]
    mean_roll = float(np.mean(roll_window))
    mean_a_lat = float(np.mean(a_lat_window))
    transfer_front = float(np.mean(front_window))
    transfer_rear = float(np.mean(rear_window))
    transfer_sum = transfer_front + transfer_rear
    if abs(mean_a_lat) < 0.1:
        roll_gradient = np.nan
    else:
        roll_gradient = float(np.degrees(mean_roll) / mean_a_lat)
    if abs(transfer_sum) < 1.0:
        front_share = np.nan
    else:
        front_share = transfer_front / transfer_sum
    roll_std = float(np.std(roll_window))
    is_steady = roll_std <= 0.02 * abs(mean_roll) or (
        roll_std <= 1e-6 and abs(mean_roll) < 1e-6
    )
    fz = np.concatenate([run.column(f"wheel.{wheel}.Fz") for wheel in WHEELS])
    deflection = np.concatenate(
        [run.column(f"wheel.{wheel}.deflection") for wheel in WHEELS]
    )
    return {
        "roll_deg": float(np.degrees(mean_roll)),
        "a_lat": mean_a_lat,
        "roll_gradient": roll_gradient,
        "transfer_front": transfer_front,
        "transfer_rear": transfer_rear,
        "front_share": front_share,
        "is_steady": bool(is_steady),
        "max_abs_roll_deg": float(np.max(np.abs(np.degrees(roll)))),
        "min_fz": float(np.min(fz)),
        "max_deflection": float(np.max(deflection)),
    }


_QUANTITIES = (
    "roll_deg",
    "a_lat",
    "roll_gradient",
    "transfer_front",
    "transfer_rear",
    "front_share",
    "max_abs_roll_deg",
    "min_fz",
    "max_deflection",
)


def table(runs, start=None, end=None) -> str:
    runs = list(runs)
    metrics = [steady(run, start, end) for run in runs]
    lines = ["quantity\t" + "\t".join(run.name for run in runs)]
    for quantity in _QUANTITIES:
        values = []
        for metric in metrics:
            values.append("not steady" if not metric["is_steady"] else str(metric[quantity]))
        lines.append(quantity + "\t" + "\t".join(values))
    return "\n".join(lines)
