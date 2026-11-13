"""Run one scenario over a range of one setting and measure each run. numpy and pandas only."""
from __future__ import annotations

import math
import os
import re
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

from . import paths
from .run import Run, load

DEFAULT_METRICS = ("maxabs:beta", "maxabs:yaw_rate", "final:yaw", "final:speed")
STATISTICS = {
    "max": lambda v: float(np.max(v)),
    "min": lambda v: float(np.min(v)),
    "maxabs": lambda v: float(np.max(np.abs(v))),
    "mean": lambda v: float(np.mean(v)),
    "final": lambda v: float(v[-1]),
    "range": lambda v: float(np.max(v) - np.min(v)),
}
HEADINGS = ("yaw",)  # world-frame headings: they jump between +pi and -pi, so they are unwrapped
_NUMBER = re.compile(r"\s*[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?")


@dataclass(frozen=True)
class SweepResult:
    scenario: str
    setting: str
    held: tuple
    metrics: tuple
    folder: Path        # the runs and summary.csv
    values: list        # the values that ran
    paths: list         # each of those runs' CSV
    runs: list          # each of those runs, loaded
    rows: list          # the summary rows: {"value": ..., "<metric>": number}
    failed: list        # (value, text) for each run that failed

    @property
    def table(self):
        return pd.DataFrame(self.rows)


def leading_number(text):
    """The number a value starts with ("45kN/m" -> 45.0); None if it has none."""
    match = _NUMBER.match(text)
    return float(match.group(0)) if match else None


def trailing_text(text):
    """What follows the leading number ("45kN/m" -> "kN/m")."""
    match = _NUMBER.match(text)
    return text[match.end():].strip() if match else text.strip()


def safe_name(text):
    """Text usable in a file name: "driver.max_yaw_rate" stays, "40deg/s" becomes "40deg_s"."""
    return re.sub(r"[^A-Za-z0-9._+-]", "_", text)


def values_of(values, range_, unit):
    """The value texts: N evenly spaced from range_ (FROM, TO, N) with `unit` written after each, else `values`."""
    if range_:
        low, high, count = float(range_[0]), float(range_[1]), int(range_[2])
        return [f"{v:g}{unit}" for v in np.linspace(low, high, count)]
    if values:
        return list(values)
    raise ValueError("give the values, or a range FROM TO N")


def check_metric(spec):
    statistic, sep, column = spec.partition(":")
    if not sep or not column or statistic not in STATISTICS:
        raise ValueError(f"metric '{spec}': expected STAT:COLUMN, STAT one of {', '.join(STATISTICS)}")


def display_column(run: Run, column):
    """A column as plotted: a heading unwrapped, angles in degrees; with its unit."""
    values = run.column(column)
    if column in HEADINGS:
        values = np.unwrap(values)  # a heading counts on past +-pi, so "final:yaw" is the turn made
    unit = run.units.get(column, "")
    if unit in ("rad", "rad/s"):
        return np.degrees(values), unit.replace("rad", "deg")
    return values, unit


def metric_value(run: Run, spec):
    """The metric `spec` ("maxabs:beta") of one run, in display units (degrees for angles), and its unit."""
    check_metric(spec)
    statistic, _, column = spec.partition(":")
    values = run.column(column)
    if column in HEADINGS:
        values = np.unwrap(values)
    value = STATISTICS[statistic](values)
    unit = run.units.get(column, "")
    if unit in ("rad", "rad/s"):
        return math.degrees(value), unit.replace("rad", "deg")
    return value, unit


def sweep(exe, scenario, setting, values, held=(), metrics=DEFAULT_METRICS, out=paths.OUT, jobs=None):
    """Run `scenario` once per value of `setting`, write the runs and summary.csv under
    out/sweeps/<scenario>__<setting>/, and return them. A run that fails is listed in `failed`; all failing is a
    ValueError. `held` is a list of NAME=VALUE settings applied to every run."""
    values, held, metrics = list(values), tuple(held), tuple(metrics)
    if not values:
        raise ValueError("no values to sweep")
    for spec in metrics:
        check_metric(spec)
    jobs = jobs or max(1, (os.cpu_count() or 2) - 1)
    folder = Path(out) / "sweeps" / f"{Path(scenario).stem}__{safe_name(setting)}"
    folder.mkdir(parents=True, exist_ok=True)

    held_args = [arg for pair in held for arg in ("--set", pair)]
    work = []
    for k, value in enumerate(values):
        csv = folder / f"{k + 1:02d}_{safe_name(value)}.csv"
        work.append((value, csv, [exe, scenario, csv, "--set", f"{setting}={value}", *held_args]))
    with ThreadPoolExecutor(max_workers=jobs) as pool:
        done = list(pool.map(lambda item: paths.run_command(item[2]), work))

    ok_values, ok_paths, runs, rows, failed = [], [], [], [], []
    for (value, csv, _), (ok, text) in zip(work, done):
        if not ok:
            failed.append((value, text))
            continue
        run = load(csv)
        row = {"value": value}
        for spec in metrics:
            row[spec], _ = metric_value(run, spec)
        ok_values.append(value)
        ok_paths.append(csv)
        runs.append(run)
        rows.append(row)
    if not rows:
        raise ValueError("every run failed: " + "; ".join(f"{v}: {t}" for v, t in failed))
    pd.DataFrame(rows).to_csv(folder / "summary.csv", index=False)
    return SweepResult(str(scenario), setting, held, metrics, folder, ok_values, ok_paths, runs, rows, failed)
