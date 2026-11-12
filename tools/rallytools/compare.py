"""Numerically compare two telemetry runs."""
from __future__ import annotations

from dataclasses import dataclass
import fnmatch

import numpy as np

from .run import Run


@dataclass(frozen=True)
class Tolerance:
    abs: float = 1e-9
    rel: float = 0.0


def tolerance_for(column, rules):
    result = Tolerance()
    for pattern, tolerance in rules:
        if fnmatch.fnmatchcase(column, pattern):
            result = tolerance
    return result


@dataclass(frozen=True)
class ColumnDiff:
    name: str
    unit: str | None
    largest: float
    t_largest: float | None
    t_first: float | None
    beyond: bool


@dataclass(frozen=True)
class Diff:
    columns: list[ColumnDiff]
    only_a: list[str]
    only_b: list[str]
    meta_changed: dict
    meta_only_a: list[str]
    meta_only_b: list[str]
    end_a: float
    end_b: float
    dt: float | None

    def differs(self, ignore_missing=False):
        if any(column.beyond for column in self.columns):
            return True
        if abs(self.end_a - self.end_b) > (self.dt or 0.0) / 2:
            return True
        return not ignore_missing and bool(self.only_a or self.only_b)


def align(a: Run, b: Run):
    if "time" not in a.data or "time" not in b.data:
        raise ValueError("both runs must contain a time column")
    start = max(float(a.data["time"].iloc[0]), float(b.data["time"].iloc[0]))
    end = min(float(a.data["time"].iloc[-1]), float(b.data["time"].iloc[-1]))
    if start > end:
        raise ValueError("runs have no overlapping time range")
    ta = a.data["time"].to_numpy(dtype=float)
    t = ta[(ta >= start) & (ta <= end)]
    columns = [column for column in a.data.columns
               if column in b.data.columns and column not in ("step", "time")]
    A = {column: a.column(column)[(ta >= start) & (ta <= end)] for column in columns}
    tb = b.data["time"].to_numpy(dtype=float)
    if np.array_equal(tb[(tb >= start) & (tb <= end)], t):
        B = {column: b.column(column)[(tb >= start) & (tb <= end)] for column in columns}
    else:
        # yaw is unwrapped first: interpolating across its jump at +-pi would invent a value.
        B = {column: np.interp(t, tb, np.unwrap(b.column(column)) if column == "yaw" else b.column(column))
             for column in columns}
    return t, columns, A, B


def _wrapped_difference(values):
    return (values + np.pi) % (2 * np.pi) - np.pi


def diff(a: Run, b: Run, rules=()):
    t, columns, values_a, values_b = align(a, b)
    result = []
    for column in columns:
        va, vb = values_a[column], values_b[column]
        used = np.ones(len(t), dtype=bool)
        if column == "beta" and a.has("speed") and b.has("speed"):
            speed_a = np.interp(t, a.data["time"], a.column("speed"))
            speed_b = np.interp(t, b.data["time"], b.column("speed"))
            used = ~((speed_a < 0.3) | (speed_b < 0.3))
        if not used.any():
            result.append(ColumnDiff(column, a.units.get(column), 0.0, None, None, False))
            continue
        with np.errstate(invalid="ignore"):
            d = vb - va
        if column == "yaw":
            d = _wrapped_difference(d)
        # A value that is NaN or infinite in one run only is a difference of infinite size.
        same = (va == vb) | (np.isnan(va) & np.isnan(vb))
        magnitude = np.where(np.isfinite(d), np.abs(d), np.where(same, 0.0, np.inf))
        magnitude[~used] = 0.0
        largest_index = int(np.argmax(magnitude))
        tolerance = tolerance_for(column, rules)
        with np.errstate(invalid="ignore"):
            limit = tolerance.abs + tolerance.rel * np.maximum(np.abs(va), np.abs(vb))
            beyond = used & ((magnitude > limit) | np.isinf(magnitude))
        first_index = np.flatnonzero(beyond)
        result.append(ColumnDiff(
            column,
            a.units.get(column),
            float(magnitude[largest_index]),
            float(t[largest_index]),
            float(t[first_index[0]]) if len(first_index) else None,
            bool(len(first_index)),
        ))
    only_a = [column for column in a.data.columns if column not in b.data.columns and column not in ("step", "time")]
    only_b = [column for column in b.data.columns if column not in a.data.columns and column not in ("step", "time")]
    meta_changed = {key: (a.meta[key], b.meta[key])
                    for key in a.meta if key in b.meta and a.meta[key] != b.meta[key]}
    step = a.dt
    if step is None and len(a.data["time"]) > 1:
        step = float(np.median(np.diff(a.data["time"].to_numpy(dtype=float))))
    return Diff(result, only_a, only_b, meta_changed,
                [key for key in a.meta if key not in b.meta],
                [key for key in b.meta if key not in a.meta],
                float(a.data["time"].iloc[-1]), float(b.data["time"].iloc[-1]), step)
