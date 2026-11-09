"""Shared helpers for the tools: the out/ folder layout, reading telemetry, and the tyre curve.

The tyre functions mirror vehicle/tyre.hpp (tyre_grip, curve_shape) so the tools can put a run's tyres on their
curve. If the C++ changes, change them here too; tools/tyre_points.py checks itself against out/rig/tyre_rig.csv
when that file exists.
"""
import math
import subprocess
import sys
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
SWEEP_PLOTS = PLOTS / "sweeps" # sweep.py: one picture per sweep
SWEEPS = OUT / "sweeps"        # sweep.py: each sweep's runs and its summary table, one folder per sweep
GIFS = OUT / "gif"             # topdown --gif (<run>.gif) and view3d --gif (<run>_3d.gif)
SNAPSHOTS = OUT / "snapshots"  # saved copies of runs/ and rig/ to compare against (run_all.py --snapshot NAME)
SCRATCH = OUT / "scratch"      # one-off experiments; nothing writes here on its own

WHEELS = ("FL", "FR", "RL", "RR")


def find_exe(given=None):
    """The rallysim executable: `given`, or the first build found under build/ (Release first)."""
    if given:
        return Path(given)
    for candidate in ("build/Release/rallysim.exe", "build/rallysim.exe", "build/Release/rallysim", "build/rallysim",
                      "build/Debug/rallysim.exe", "build/Debug/rallysim"):
        path = ROOT / candidate
        if path.exists():
            return path
    sys.exit("rallysim not found under build/; build it first or pass --exe")


def run_command(command):
    """Run one command from the sim folder; (ok, text) with its output."""
    done = subprocess.run([str(c) for c in command], cwd=ROOT, capture_output=True, text=True)
    return done.returncode == 0, (done.stdout + done.stderr).strip()


def read_units(path):
    """{column: unit} from line 1 of a telemetry file ("name[unit]" after "units:")."""
    with open(path, encoding="utf-8") as f:
        first = f.readline()
    units = {}
    if "units:" in first:
        for token in first.split("units:", 1)[1].split():
            name, _, unit = token.partition("[")
            units[name] = unit.rstrip("]")
    return units


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


# ---- The tyre, mirrored from vehicle/tyre.hpp ----------------------------------------------------------------
# A tyre is a dict of the numbers on line 1 of a four-wheel file (four_wheel_meta), without the axle suffix.
# Defaults are vehicle/tyre.hpp's, for files written before a key was on line 1.

TMEASY_FIELDS = ("stiffness", "peak_slip", "sliding_slip", "sliding_ratio")
TMEASY_DEFAULTS = {  # tmeasy_x and tmeasy_y in tyre_params: at the nominal load (1) and at twice it (2)
    "x": {1: (90000.0, 0.090, 0.8, 0.75), 2: (160000.0, 0.095, 0.8, 0.75)},
    "y": {1: (30000.0, 0.165, 0.7, 0.85), 2: (55000.0, 0.175, 0.7, 0.85)},
}
TYRE_DEFAULTS = {"mu": 1.0, "k": 0.1, "fz0": 3000.0, "cx": 90000.0, "calpha": 30000.0, "curve": 0.0,
                 "peak_slip": 2.0, "sliding_slip": 4.0, "sliding_ratio": 0.8}
for _axis, _points in TMEASY_DEFAULTS.items():
    for _n, _values in _points.items():
        for _field, _value in zip(TMEASY_FIELDS, _values):
            TYRE_DEFAULTS[f"tmeasy_{_axis}_{_field}{_n}"] = _value
CURVES = {0: "linear", 1: "peak_slide", 2: "tmeasy"}


def tyre_from_meta(meta, axle):
    """The tyre of one axle ('front' or 'rear') from line 1, with defaults for missing keys."""
    tyre = {key: meta.get(f"{key}_{axle}", default) for key, default in TYRE_DEFAULTS.items()}
    tyre["missing"] = [key for key in TYRE_DEFAULTS if f"{key}_{axle}" not in meta]
    return tyre


def curve_name(tyre):
    return CURVES.get(int(tyre["curve"]), "?")


def grip(tyre, fz):
    """mu(Fz) Fz with load sensitivity, zero when unloaded (tyre_grip): the peak of every curve."""
    fz = np.asarray(fz, dtype=float)
    mu = tyre["mu"] * np.maximum(1 - tyre["k"] * (fz - tyre["fz0"]) / tyre["fz0"], 0)
    return np.where(fz > 0, mu * fz, 0.0)


def shape(s, p, q, r):
    """peak_slide_shape: force over peak force at s (slope 1 at the start): cubic to 1 at p, half-cosine to r at q."""
    s, p, q, r = (np.asarray(v, dtype=float) for v in (s, p, q, r))
    a, b = (3 - 2 * p) / p**2, (p - 2) / p**3
    rise = s + a * s**2 + b * s**3
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.clip((s - p) / (q - p), 0, 1)
    fall = r + (1 - r) * (1 + np.cos(math.pi * u)) / 2
    return np.where(s <= p, rise, np.where(s < q, fall, r))


def curve_shape(tyre, s):
    """Force over grip at normalised slip s >= 0 for a linear or peak_slide tyre (min(s, 1) for linear)."""
    if curve_name(tyre) == "peak_slide":
        return shape(s, tyre["peak_slip"], tyre["sliding_slip"], tyre["sliding_ratio"])
    return np.minimum(np.asarray(s, dtype=float), 1.0)


def tmeasy_at(tyre, axis, fz):
    """tmeasy_at: one direction's (stiffness, peak slip, sliding slip, sliding ratio) at load fz."""
    f = np.maximum(np.asarray(fz, dtype=float), 0) / tyre["fz0"]
    one = [tyre[f"tmeasy_{axis}_{field}1"] for field in TMEASY_FIELDS]
    two = [tyre[f"tmeasy_{axis}_{field}2"] for field in TMEASY_FIELDS]
    stiffness = np.maximum(f * (2 * one[0] - 0.5 * two[0] - (one[0] - 0.5 * two[0]) * f), 0)
    linear = lambda i: one[i] + (two[i] - one[i]) * (f - 1)
    peak = np.maximum(linear(1), 1e-4)
    sliding = np.maximum(linear(2), 1.01 * peak)
    ratio = np.clip(linear(3), 0, 1)
    return stiffness, peak, sliding, ratio


def pure_slip_curve(tyre, axis, fz):
    """pure_slip_curve: (stiffness, peak slip, sliding slip, sliding ratio) of one direction, any curve."""
    if curve_name(tyre) == "tmeasy":
        return tmeasy_at(tyre, axis, fz)
    g = grip(tyre, fz)
    c = tyre["cx"] if axis == "x" else tyre["calpha"]
    if curve_name(tyre) == "peak_slide":
        return c, tyre["peak_slip"] * g / c, tyre["sliding_slip"] * g / c, tyre["sliding_ratio"]
    return c, g / c, g / c, 1.0


def tmeasy_combine(tyre, sx, sy, fz):
    """tmeasy_combine: (sigma, ux, uy, p, q, r) for slips sx, sy at load fz (arrays work)."""
    kx, mx, slx, rx = tmeasy_at(tyre, "x", fz)
    ky, my, sly, ry = tmeasy_at(tyre, "y", fz)
    g = grip(tyre, fz)
    with np.errstate(divide="ignore", invalid="ignore"):
        nx, ny = np.asarray(sx, dtype=float) / mx, np.asarray(sy, dtype=float) / my
        sigma = np.hypot(nx, ny)
        cx = np.where(sigma > 0, nx / sigma, 1.0)
        cy = np.where(sigma > 0, ny / sigma, 0.0)
        slope = np.hypot(kx * mx * cx, ky * my * cy)
        p = np.where(g > 0, np.clip(slope / g, 1.5, 3.0), 2.0)
        sliding = np.hypot(slx / mx * cx, sly / my * cy)
        ratio = np.hypot(rx * cx, ry * cy)
        lx, ly = kx * np.asarray(sx, dtype=float), ky * np.asarray(sy, dtype=float)
        l = np.hypot(lx, ly)
        ux, uy = np.where(l > 0, lx / l, cx), np.where(l > 0, ly / l, cy)
    return sigma, ux, uy, p, sliding * p, ratio


def tyre_forces(tyre, sx, sy, fz):
    """tyre_forces: (fx, fy) in the wheel frame for slip ratio sx, lateral slip sy, load fz (arrays work)."""
    sx, sy = np.asarray(sx, dtype=float), np.asarray(sy, dtype=float)
    g = grip(tyre, fz)
    name = curve_name(tyre)
    with np.errstate(divide="ignore", invalid="ignore"):
        if name == "tmeasy":
            sigma, ux, uy, p, q, r = tmeasy_combine(tyre, sx, sy, fz)
            force = np.where(sigma > 0, g * shape(sigma * p, p, q, r), 0.0)
            return force * ux, force * uy
        if name == "peak_slide":
            kx, ky = tyre["cx"] * sx / g, tyre["calpha"] * sy / g
            s = np.hypot(kx, ky)
            force = np.where(s > 0, g * curve_shape(tyre, s), 0.0)
            return np.where(s > 0, force * kx / s, 0.0), np.where(s > 0, force * ky / s, 0.0)
        fx, fy = tyre["cx"] * sx, tyre["calpha"] * sy
        size = np.hypot(fx, fy)
        scale = np.where(size > g, g / size, 1.0)
        return fx * scale, fy * scale


def normalised_slip(tyre, slip_ratio, lateral_slip, fz):
    """The combined slip each curve reads its force at, NaN where unloaded. Linear and peak_slide: the length
    of (Cx sx, C_alpha sy) / grip (1 = where the linear tyre reaches the grip). tmeasy: sigma, the length of the
    slips divided by their own peak slips (1 = the peak, in every direction)."""
    g = grip(tyre, fz)
    with np.errstate(divide="ignore", invalid="ignore"):
        if curve_name(tyre) == "tmeasy":
            sigma = tmeasy_combine(tyre, slip_ratio, lateral_slip, fz)[0]
            return np.where(g > 0, sigma, np.nan)
        kx = np.where(g > 0, tyre["cx"] * np.asarray(slip_ratio) / g, np.nan)
        ky = np.where(g > 0, tyre["calpha"] * np.asarray(lateral_slip) / g, np.nan)
    return np.hypot(kx, ky)


def slip_axis_label(tyre):
    if curve_name(tyre) == "tmeasy":
        return "normalised slip sigma  (each slip over its own peak slip: 1 = the peak)"
    return "normalised slip s  (1 = where the linear tyre reaches the grip)"


def reference_curves(tyre, edge, axle=""):
    """Curves to draw under a run's points, on normalised_slip's axis, as (label, s, force over grip). Linear and
    peak_slide have one curve for every load and direction. tmeasy has one per direction; drawn at the nominal load
    (other loads differ a little)."""
    s = np.linspace(0, edge, 400)
    name = curve_name(tyre)
    suffix = f" ({axle})" if axle else ""
    if name != "tmeasy":
        return [(name + suffix, s, curve_shape(tyre, s))]
    out = []
    for axis, word in (("x", "along"), ("y", "across")):
        k, m, sl, r = tmeasy_at(tyre, axis, tyre["fz0"])
        p = np.clip(k * m / grip(tyre, tyre["fz0"]), 1.5, 3.0)
        out.append((f"tmeasy {word}{suffix}, at Fz0", s, shape(s * p, p, sl / m * p, r)))
    return out


def marks(tyre):
    """(position, label) on normalised_slip's axis worth a line: the peak and full sliding."""
    name = curve_name(tyre)
    if name == "peak_slide":
        return [(tyre["peak_slip"], "peak"), (tyre["sliding_slip"], "full sliding")]
    if name == "tmeasy":
        _, mx, slx, _ = tmeasy_at(tyre, "x", tyre["fz0"])
        _, my, sly, _ = tmeasy_at(tyre, "y", tyre["fz0"])
        return [(1.0, "peak"), (float(slx / mx), "full sliding along"), (float(sly / my), "full sliding across")]
    return [(1.0, "linear reaches the grip")]


def axis_edge(tyre):
    """How far along normalised_slip's axis to draw: past full sliding, so locked wheels pin to the edge."""
    name = curve_name(tyre)
    if name == "peak_slide":
        return max(1.6 * tyre["sliding_slip"], 2.5)
    if name == "tmeasy":
        return 1.2 * max(position for position, _ in marks(tyre))
    return 2.5


def check_against_rig(path=None):
    """Largest difference (N) between this file's tyre_forces and the C++ one, over every row of the rig
    (every curve, sweep and load); None if there is no rig file."""
    path = Path(path) if path else RIG / "tyre_rig.csv"
    if not path.exists():
        return None
    rig = pd.read_csv(path, comment="#")
    curves = {"linear": {}, "brush": {"curve": 1, "peak_slip": 3.0, "sliding_ratio": 1.0},
              "peak_slide": {"curve": 1}, "tmeasy": {"curve": 2}}
    worst = 0.0
    for name, overrides in curves.items():
        rows = rig[rig["curve"] == name]
        if rows.empty:
            continue
        tyre = dict(TYRE_DEFAULTS, **overrides)
        sy = np.tan(np.radians(rows["slip_angle_deg"].to_numpy()))
        fx, fy = tyre_forces(tyre, rows["slip_ratio"].to_numpy(), sy, rows["fz"].to_numpy())
        worst = max(worst, float(np.max(np.abs(fx - rows["fx"].to_numpy()))),
                    float(np.max(np.abs(fy - rows["fy"].to_numpy()))))
    return worst


def label_of(path):
    """A short name for a run file: 'fw_brake', or 'linear/fw_brake' for a snapshot."""
    path = Path(path).resolve()
    try:
        parts = path.relative_to(SNAPSHOTS).parts
        return f"{parts[0]}/{path.stem}"
    except ValueError:
        return path.stem
