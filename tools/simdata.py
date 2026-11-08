"""Shared helpers for the tools: the out/ folder layout, reading telemetry, and the tyre curve.

The tyre functions mirror vehicle/tyre.hpp (tyre_grip, curve_shape) so the tools can put a run's tyres on their
curve. If the C++ changes, change them here too; tools/tyre_points.py checks itself against out/rig/tyre_rig.csv
when that file exists.
"""
import math
from pathlib import Path

import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent  # the sim/ folder
OUT = ROOT / "out"
RUNS = OUT / "runs"            # one telemetry CSV per scenario (rallysim <scenario>)
RIG = OUT / "rig"              # tyre_rig.csv (rallysim tyre_rig)
PLOTS = OUT / "plots"
VIEWS = PLOTS / "views"        # topdown --static, one per run
TYRES = PLOTS / "tyres"        # the rig plot, and each four-wheel run's tyres on their curve
COMPARE = PLOTS / "compare"    # compare.py
GIFS = OUT / "gif"             # topdown --gif, one per run
SNAPSHOTS = OUT / "snapshots"  # saved copies of runs/ and rig/ to compare against (run_all.py --snapshot NAME)
SCRATCH = OUT / "scratch"      # one-off experiments; nothing writes here on its own

WHEELS = ("FL", "FR", "RL", "RR")


def read_meta(path):
    """key=value pairs on line 1 of a telemetry file, between dt and the units (numbers only)."""
    with open(path, encoding="utf-8") as f:
        first = f.readline()
    meta = {}
    if first.startswith("#"):
        for token in first[1:].split("units:")[0].split():
            key, sep, value = token.partition("=")
            if sep:
                try:
                    meta[key] = float(value)
                except ValueError:
                    pass
    return meta


def read_run(path):
    """(data, meta) of a telemetry file."""
    return pd.read_csv(path, comment="#"), read_meta(path)


HEADINGS = ("yaw",)  # world-frame headings: atan2 of the orientation, so they jump between +pi and -pi


def unwrap_headings(data):
    """Remove the 2 pi jumps from the heading columns, in place, so a car going round a circle shows a heading
    that keeps climbing (as the bicycle's integrated yaw does) instead of a sawtooth. Same heading, plotted
    continuously; a file that is already continuous is unchanged."""
    for column in HEADINGS:
        if column in data.columns:
            data[column] = np.unwrap(data[column].to_numpy(dtype=float))
    return data


def is_four_wheel(data):
    return "wheel.FL.slipRatio" in data.columns


# Tyre defaults (vehicle/tyre.hpp), for files written before the tyre keys were on line 1.
TYRE_DEFAULTS = {"mu": 1.0, "k": 0.1, "fz0": 3000.0, "cx": 90000.0, "calpha": 30000.0, "curve": 0.0,
                 "peak_slip": 2.0, "sliding_slip": 4.0, "sliding_ratio": 0.8}
CURVES = {0: "linear", 1: "peak_slide"}


def tyre_from_meta(meta, axle):
    """The tyre of one axle ('front' or 'rear') from line 1, with defaults for missing keys."""
    tyre = {key: meta.get(f"{key}_{axle}", default) for key, default in TYRE_DEFAULTS.items()}
    tyre["missing"] = [key for key in TYRE_DEFAULTS if f"{key}_{axle}" not in meta]
    return tyre


def grip(tyre, fz):
    """mu(Fz) Fz with load sensitivity, zero when unloaded (tyre_grip)."""
    fz = np.asarray(fz, dtype=float)
    mu = tyre["mu"] * np.maximum(1 - tyre["k"] * (fz - tyre["fz0"]) / tyre["fz0"], 0)
    return np.where(fz > 0, mu * fz, 0.0)


def curve_shape(tyre, s):
    """Force over grip at normalised slip s >= 0 (curve_shape, or min(s, 1) for the linear tyre)."""
    s = np.asarray(s, dtype=float)
    if CURVES.get(int(tyre["curve"])) != "peak_slide":
        return np.minimum(s, 1.0)
    p, q, r = tyre["peak_slip"], tyre["sliding_slip"], tyre["sliding_ratio"]
    a, b = (3 - 2 * p) / p**2, (p - 2) / p**3
    rise = s + a * s**2 + b * s**3
    u = np.clip((s - p) / (q - p), 0, 1)
    fall = r + (1 - r) * (1 + np.cos(math.pi * u)) / 2
    return np.where(s <= p, rise, np.where(s < q, fall, r))


def normalised_slip(tyre, slip_ratio, lateral_slip, fz):
    """The combined slip the curve is read at: (Cx sx, C_alpha sy) / grip, its length. NaN where unloaded."""
    g = grip(tyre, fz)
    with np.errstate(divide="ignore", invalid="ignore"):
        kx = np.where(g > 0, tyre["cx"] * np.asarray(slip_ratio) / g, np.nan)
        ky = np.where(g > 0, tyre["calpha"] * np.asarray(lateral_slip) / g, np.nan)
    return np.hypot(kx, ky)


def check_against_rig(path=None):
    """Largest difference (N) between this file's curve_shape and the C++ one, from the rig's peak_slide
    and brush curves at their nominal load; None if there is no rig file."""
    path = Path(path) if path else RIG / "tyre_rig.csv"
    if not path.exists():
        return None
    rig = pd.read_csv(path, comment="#")
    worst = 0.0
    for name, overrides in (("peak_slide", {"curve": 1}), ("brush", {"curve": 1, "peak_slip": 3.0, "sliding_ratio": 1.0})):
        tyre = dict(TYRE_DEFAULTS, **overrides)
        rows = rig[(rig["curve"] == name) & (rig["sweep"] == "x") & (rig["fz"] == tyre["fz0"])]
        if rows.empty:
            continue
        s = normalised_slip(tyre, rows["slip_ratio"].to_numpy(), 0.0, tyre["fz0"])
        expected = grip(tyre, tyre["fz0"]) * curve_shape(tyre, s) * np.sign(rows["slip_ratio"].to_numpy())
        worst = max(worst, float(np.max(np.abs(expected - rows["fx"].to_numpy()))))
    return worst


def label_of(path):
    """A short name for a run file: 'fw_brake', or 'linear/fw_brake' for a snapshot."""
    path = Path(path).resolve()
    try:
        parts = path.relative_to(SNAPSHOTS).parts
        return f"{parts[0]}/{path.stem}"
    except ValueError:
        return path.stem
