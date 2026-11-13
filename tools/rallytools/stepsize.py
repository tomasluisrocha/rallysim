"""Step-size report: one scenario run at dt, dt/2 and dt/4. numpy and pandas only."""
from __future__ import annotations

from dataclasses import dataclass
import math

from .compare import Tolerance, diff
from .run import Run

_EXACT = [("*", Tolerance(0.0, 0.0))]
_HALVING = 0.6


@dataclass(frozen=True)
class Row:
    name: str
    unit: str | None
    d1: float
    d2: float
    ratio: float | None
    verdict: str


def _check_steps(a: Run, b: Run, c: Run):
    for label, run in (("A", a), ("B", b), ("C", c)):
        if run.dt is None:
            raise ValueError(f"run {label} has no dt on line 1")
    if abs(b.dt - a.dt / 2) > 1e-12 or abs(c.dt - a.dt / 4) > 1e-12:
        raise ValueError(f"steps must be dt, dt/2 and dt/4, got {a.dt!r}, {b.dt!r} and {c.dt!r}")


def _verdict(d1, d2, ratio, abs_tol):
    if math.isinf(d1) or math.isinf(d2):
        return "not converging"
    if d1 <= abs_tol and d2 <= abs_tol:
        return "equal"
    return "converging" if ratio is not None and ratio <= _HALVING else "not converging"


def report(a: Run, b: Run, c: Run, abs_tol: float = 1e-9) -> list[Row]:
    _check_steps(a, b, c)
    first = {column.name: column for column in diff(a, b, _EXACT).columns}
    second = {column.name: column for column in diff(b, c, _EXACT).columns}
    rows = []
    for name, one in first.items():
        if name not in second:
            continue
        d1, d2 = one.largest, second[name].largest
        ratio = None if d1 <= abs_tol else d2 / d1
        rows.append(Row(name, one.unit, d1, d2, ratio, _verdict(d1, d2, ratio, abs_tol)))
    return rows


def text(rows: list[Row], show_all: bool = False) -> str:
    bad = sorted((row for row in rows if row.verdict == "not converging"), key=lambda row: -row.d2)
    chosen = list(bad)
    if show_all:
        chosen += sorted((row for row in rows if row.verdict != "not converging"), key=lambda row: -row.d2)
    lines = ["column\td1\td2\tratio\tverdict"]
    for row in chosen:
        ratio = "n/a" if row.ratio is None else f"{row.ratio:.4g}"
        lines.append(f"{row.name}\t{row.d1:.6g}\t{row.d2:.6g}\t{ratio}\t{row.verdict}")
    return "\n".join(lines)
