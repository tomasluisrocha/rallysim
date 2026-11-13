"""Derived channels for four-wheel telemetry runs."""
from __future__ import annotations

import numpy as np

from .run import Run, WHEELS


NAMES = {
    "transfer.front": "N",
    "transfer.rear": "N",
    "transfer.front_share": "-",
    "antiroll.front": "N",
    "antiroll.rear": "N",
    "load.total": "N",
    **{f"wheel.{wheel}.gripUse": "-" for wheel in WHEELS},
    "yaw.unwrapped": "rad",
}


def _four_wheel(run: Run) -> None:
    if run.kind != "four_wheel":
        raise ValueError(f"run '{run.name}': derived channels require a four_wheel run")


def derived(run: Run, name: str) -> np.ndarray:
    """Return one derived channel, calculated row by row."""
    _four_wheel(run)
    if name not in NAMES:
        raise ValueError(f"unknown derived channel '{name}'")
    if name == "transfer.front":
        return (run.column("wheel.FR.Fz") - run.column("wheel.FL.Fz")) / 2.0
    if name == "transfer.rear":
        return (run.column("wheel.RR.Fz") - run.column("wheel.RL.Fz")) / 2.0
    if name == "antiroll.front":
        left = run.column("wheel.FL.arbF")
        return (run.column("wheel.FR.arbF") - left) / 2.0
    if name == "antiroll.rear":
        left = run.column("wheel.RL.arbF")
        return (run.column("wheel.RR.arbF") - left) / 2.0
    if name == "transfer.front_share":
        front = derived(run, "transfer.front")
        total = front + derived(run, "transfer.rear")
        with np.errstate(divide="ignore", invalid="ignore"):
            return np.where(np.abs(total) < 1.0, np.nan, front / total)
    if name == "load.total":
        return sum(run.column(f"wheel.{wheel}.Fz") for wheel in WHEELS)
    if name.startswith("wheel.") and name.endswith(".gripUse"):
        wheel = name.split(".")[1]
        force = np.hypot(run.column(f"wheel.{wheel}.Fx"), run.column(f"wheel.{wheel}.Fy"))
        grip = run.column(f"wheel.{wheel}.grip")
        with np.errstate(divide="ignore", invalid="ignore"):
            return np.where(grip > 0.0, force / grip, np.nan)
    if name == "yaw.unwrapped":
        return np.unwrap(run.column("yaw"))
    raise AssertionError(name)
