"""Events of a four-wheel run and landing metrics. numpy only."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .run import WHEELS

_AXLE = {"FL": "front", "FR": "front", "RL": "rear", "RR": "rear"}
_BAND = 0.05
_QUIET_WINDOW = 0.5
_QUIET_VARIATION = 0.01
_REBOUND_MIN = 0.02
_WINDOW_SLACK = 1e-9


@dataclass(frozen=True)
class Landing:
    t_touchdown: float
    airborne_time: float
    peak_fz: dict
    peak_fz_nominal: dict
    peak_travel: dict
    travel_margin: dict
    peak_stop: dict
    stop_time: dict
    settling_time: Optional[float]
    rebounds: int
    overshoot: float


@dataclass(frozen=True)
class Event:
    kind: str
    wheel: str
    t_start: float
    t_end: float


_LOCK_SLIP = -0.95  # A wheel is locked at or below this slip ratio.
_SPIN_SLIP = 0.2  # A wheel is spinning at or above this slip ratio.
_MIN_EVENT_SPEED = 1.0  # Lock and spin require this contact speed in m/s.
_MIN_EVENT_TIME = 0.05  # Lock, spin and lift events shorter than this are noise.
_EVENT_EPSILON = 1e-9  # Keep events exactly at the minimum duration.


def _stretches(fz):
    """(start, end) row indices of the all-wheels-airborne stretches; end is len(fz) if the run ends airborne."""
    fz = np.asarray(fz, dtype=float)
    air = np.all(fz <= 0.0, axis=1)
    edges = np.diff(np.concatenate(([0], air.astype(int), [0])))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def events(run, min_time=_MIN_EVENT_TIME):
    """Return wheel events in a four-wheel run, ordered by time and wheel."""
    if run.kind != "four_wheel":
        raise ValueError(f"run '{run.name}': events need a four-wheel run, not {run.kind}")
    if "wheel_radius" not in run.meta:
        raise ValueError(f"run '{run.name}': no line-1 key 'wheel_radius'")

    t = run.column("time")
    speed = run.column("speed")
    radius = run.meta["wheel_radius"]
    found = []
    conditions = {}
    for wheel in WHEELS:
        prefix = f"wheel.{wheel}."
        fz = run.column(prefix + "Fz")
        slip = run.column(prefix + "slipRatio")
        omega = run.column(prefix + "omega")
        conditions[wheel] = {
            "lock": (slip <= _LOCK_SLIP) & (speed >= _MIN_EVENT_SPEED) & (fz > 0.0),
            "spin": (slip >= _SPIN_SLIP) & (omega * radius >= _MIN_EVENT_SPEED) & (fz > 0.0),
            "lift": fz <= 0.0,
        }
        if run.has(prefix + "stopF"):
            stop = run.column(prefix + "stopF")
            conditions[wheel]["bump_stop"] = stop > 0.0
            conditions[wheel]["droop_stop"] = stop < 0.0

    for wheel in WHEELS:
        for kind, condition in conditions[wheel].items():
            edges = np.diff(np.concatenate(([False], condition, [False])).astype(int))
            for start, end in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
                t_start = float(t[start])
                t_end = float(t[end] if end < len(t) else t[-1])
                if kind not in ("bump_stop", "droop_stop") and t_end - t_start < min_time - _EVENT_EPSILON:
                    continue
                found.append(Event(kind, wheel, t_start, t_end))

    wheel_order = {wheel: i for i, wheel in enumerate(WHEELS)}
    return sorted(found, key=lambda event: (event.t_start, wheel_order[event.wheel], event.kind))


def airborne(t, fz):
    t = np.asarray(t, dtype=float)
    n = len(t)
    return [(float(t[i]), float(t[j] if j < n else t[-1])) for i, j in _stretches(fz)]


def settling_time(t, fz):
    t = np.asarray(t, dtype=float)
    fz = np.asarray(fz, dtype=float)
    end = fz[-1]
    out = np.any(np.abs(fz - end) > _BAND * np.abs(end), axis=1)
    if out[-1]:
        return None
    tail = fz[t >= t[-1] - _QUIET_WINDOW]
    spread = tail.max(axis=0) - tail.min(axis=0)
    mean = tail.mean(axis=0)
    with np.errstate(divide="ignore", invalid="ignore"):
        variation = np.where(spread == 0.0, 0.0, spread / mean)
    if np.any(np.abs(variation) > _QUIET_VARIATION) or not np.all(np.isfinite(variation)):
        return None
    rows = np.flatnonzero(out)
    if rows.size == 0:
        return 0.0
    return float(t[rows[-1] + 1] - t[0])


def rebounds(z):
    z = np.asarray(z, dtype=float)
    z_end = z[-1]
    low = int(np.argmin(z))
    d0 = z_end - z[low]
    if d0 <= 0.0:
        return 0, 0.0
    heights = [z[i] - z_end for i in range(low + 1, len(z) - 1)
               if z[i] > z[i - 1] and z[i] >= z[i + 1] and z[i] - z_end > _REBOUND_MIN * d0]
    if not heights:
        return 0, 0.0
    return len(heights), float(heights[0] / d0)


def landings(run, window=None):
    """Landings of a four-wheel run. With a window (s), a landing's rows end window after touchdown."""
    if window is not None and not window > 0.0:
        raise ValueError(f"landing window must be positive, not {window}")
    if run.kind != "four_wheel":
        raise ValueError(f"run '{run.name}': landings need a four-wheel run, not {run.kind}")
    t = run.column("time")
    fz = np.column_stack([run.column(f"wheel.{w}.Fz") for w in WHEELS])
    z = run.column("pos.z")
    n = len(t)
    stretches = _stretches(fz)
    found = []
    for k, (start, end) in enumerate(stretches):
        if end >= n:
            continue
        stop = stretches[k + 1][0] if k + 1 < len(stretches) else n
        if window is not None:
            stop = end + int(np.searchsorted(t[end:stop], t[end] + window + _WINDOW_SLACK, side="right"))
        rows = slice(end, stop)
        peak_fz, nominal, travel, margin, peak_stop, stop_time = {}, {}, {}, {}, {}, {}
        for i, w in enumerate(WHEELS):
            axle = _AXLE[w]
            peak_fz[w] = float(fz[rows, i].max())
            fz0 = run.meta.get(f"fz0_{axle}")
            nominal[w] = None if fz0 is None else peak_fz[w] / fz0
            rest = run.meta.get(f"rest_deflection_{axle}")
            deflection = run.column(f"wheel.{w}.deflection")[rows]
            travel[w] = None if rest is None else float(deflection.max() - rest)
            bump = run.meta.get(f"bump_travel_{axle}")
            margin[w] = (bump - travel[w]
                         if run.vertical == "unsprung" and travel[w] is not None and bump is not None
                         else None)
            stop_f = run.column(f"wheel.{w}.stopF")[rows] if run.has(f"wheel.{w}.stopF") else np.zeros(0)
            positive = stop_f[stop_f > 0.0]
            peak_stop[w] = float(positive.max()) if positive.size else 0.0
            stop_time[w] = None if run.dt is None else float(positive.size * run.dt)
        count, overshoot = rebounds(z[rows])
        found.append(Landing(float(t[end]), float(t[end] - t[start]), peak_fz, nominal, travel, margin,
                             peak_stop, stop_time, settling_time(t[rows], fz[rows]), count, overshoot))
    return found
