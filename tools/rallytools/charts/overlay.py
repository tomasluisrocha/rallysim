"""Overlay of any number of runs: their paths, chosen channels, and each run minus the first."""
from __future__ import annotations

from dataclasses import replace

import numpy as np
from matplotlib.figure import Figure

from ..compare import align
from ..run import Run
from ..tyre import CURVES

DEFAULTS = {
    "four_wheel": ["speed", "yaw_rate", "beta", "a_lat", "pitch"],
    "single_track": ["speed", "yaw_rate", "beta", "a_lat"],
    "bicycle": ["speed", "yaw_rate", "beta", "a_lat"],
    "rigid_body": ["pos.z", "vel.x", "vel.z", "omega.z"],
}
ANGLES = {"yaw", "beta"}  # differences wrapped to [-pi, pi)
STILL_SPEED = 0.3         # m/s: below this beta is not compared
INPUTS = ("steer", "throttle", "brake", "handbrake")  # the driver's columns
INPUT_TOL = 1e-9          # inputs closer than this are the same
SHOWN = 2                 # differing line-1 parameters named in a label
CURVE_KEYS = ("curve", "curve_front", "curve_rear")  # line-1 values that are tyre curve codes
COLOURS = ["tab:blue", "tab:red", "tab:green", "tab:orange", "tab:purple", "tab:brown", "tab:pink", "tab:olive"]


def _letter(index):
    return chr(ord("A") + index) if index < 26 else str(index)


def _value(key, value):
    """A line-1 value as printed: tyre curves by name, other numbers with :g."""
    if key in CURVE_KEYS:
        return CURVES.get(int(value), "?")
    return f"{value:g}"


def _described(runs, origins):
    """Each origin followed by the line-1 parameters that differ from the first run, as "origin: key (value)", at
    most SHOWN of them and then " +N more"; a run with none differing keeps its origin. For a later run these
    are the keys whose value differs from the first run's; the first run shows the values of the keys that
    differ in any later run. Keys are in the first run's line-1 order; only keys on both lines count."""
    first = runs[0].meta
    differing = [[k for k in first if k in run.meta and run.meta[k] != first[k]] for run in runs]
    differing[0] = [k for k in first if any(k in keys for keys in differing[1:])]
    labels = []
    for run, origin, keys in zip(runs, origins, differing):
        if not keys:
            labels.append(origin)
            continue
        text = ", ".join(f"{k} ({_value(k, run.meta[k])})" for k in keys[:SHOWN])
        more = f" +{len(keys) - SHOWN} more" if len(keys) > SHOWN else ""
        labels.append(f"{origin}: {text}{more}")
    return labels


def _prepare(runs, channels, labels, describe=None):
    """describe: whether labels (or the run names, when there are none) are origins that get the differing
    line-1 parameters appended; None means when no labels are given."""
    runs = list(runs)
    if len(runs) < 2:
        raise ValueError("overlay needs at least two runs")
    if not channels:
        kinds = {run.kind for run in runs}
        if len(kinds) > 1:
            raise ValueError("runs of different kinds (" + ", ".join(sorted(kinds)) + "): name the channels")
        channels = DEFAULTS[runs[0].kind]
    channels = list(channels)
    missing = [c for c in channels if not all(run.has(c) for run in runs)]
    if missing:
        raise ValueError(f"not in every run: {', '.join(missing)}")
    if describe is None:
        describe = labels is None
    if labels is None:
        labels = [run.name for run in runs]
        if len(set(labels)) < len(labels):
            labels = [_letter(i) for i in range(len(runs))]
    labels = list(labels)
    if len(labels) != len(runs):
        raise ValueError(f"{len(labels)} labels for {len(runs)} runs")
    if describe:
        labels = _described(runs, labels)
    # yaw is drawn continuously, not as a sawtooth; differences are still wrapped below
    unwrapped = []
    for run in runs:
        if run.has("yaw"):
            data = run.data.copy()
            data["yaw"] = np.unwrap(run.column("yaw"))
            run = replace(run, data=data)
        unwrapped.append(run)
    return unwrapped, channels, labels


def _pair(first: Run, other: Run, channels):
    """(t, rows) with one row per channel: (a, b, difference b - a) on the first run's times."""
    t, _, values_a, values_b = align(first, other)
    rows = []
    for channel in channels:
        va = values_a[channel].copy()
        vb = values_b[channel].copy()
        if channel == "beta" and "speed" in values_a:
            # sideslip means nothing at a standstill (it flips between 0 and pi): leave those moments out
            va[values_a["speed"] < STILL_SPEED] = np.nan
            vb[values_b["speed"] < STILL_SPEED] = np.nan
        diff = vb - va
        if channel in ANGLES:
            diff = (diff + np.pi) % (2 * np.pi) - np.pi  # a heading of pi and -pi is the same heading
        rows.append((va, vb, diff))
    return t, rows


def _inputs_note(first: Run, other: Run):
    """The line "inputs differ (closed loop): <column> from <time> s" for the driver column that differs
    earliest between the runs (compared on the first run's times); None when they agree or a run has no
    input columns."""
    names = [name for name in INPUTS if first.has(name) and other.has(name)]
    if not names:
        return None
    t, _, values_a, values_b = align(first, other)
    best = None
    for name in names:
        va, vb = values_a[name], values_b[name]
        with np.errstate(invalid="ignore"):
            differs = (np.abs(vb - va) > INPUT_TOL) | (np.isnan(va) != np.isnan(vb))
        if differs.any():
            when = float(t[int(np.argmax(differs))])
            if best is None or when < best[1]:
                best = (name, when)
    return None if best is None else f"inputs differ (closed loop): {best[0]} from {best[1]:g} s"


def table(runs, channels=None, labels=None, paths=None, describe=None) -> str:
    """The text table: per later run, the labels, the line-1 facts that differ, a note when the driver inputs
    differ and, per channel, the largest difference from the first run and both final values. paths (optional)
    are shown after the labels. describe: see _prepare."""
    runs, channels, labels = _prepare(runs, channels, labels, describe)
    first = runs[0]
    blocks = []
    for index in range(1, len(runs)):
        other = runs[index]
        t, rows = _pair(first, other, channels)
        a, b = _letter(0), _letter(index)
        where = [f"  ({paths[0]})", f"  ({paths[index]})"] if paths else ["", ""]
        lines = [f"{a} = {labels[0]}{where[0]}", f"{b} = {labels[index]}{where[1]}"]
        meta_a, meta_b = first.meta, other.meta
        changed = sorted(k for k in set(meta_a) & set(meta_b) if meta_a[k] != meta_b[k])
        only_a, only_b = sorted(set(meta_a) - set(meta_b)), sorted(set(meta_b) - set(meta_a))
        if changed:
            lines.append("line-1 facts that differ: "
                         + ", ".join(f"{k} {_value(k, meta_a[k])} -> {_value(k, meta_b[k])}" for k in changed))
        for keys, name in ((only_a, a), (only_b, b)):
            if keys:  # a newer file has more keys: count them rather than list dozens
                shown = ", ".join(keys[:4]) + (", ..." if len(keys) > 4 else "")
                lines.append(f"line-1 facts only in {name}: {len(keys)} ({shown})")
        if not (changed or only_a or only_b):
            lines.append("line-1 facts: the same")
        note = _inputs_note(first, other)
        if note:
            lines.append(note)
        end_a = float(first.data["time"].iloc[-1])
        end_b = float(other.data["time"].iloc[-1])
        if end_a != end_b:
            lines.append(f"note: {a} ends at {end_a:g} s, {b} at {end_b:g} s; compared over {t[0]:g} to {t[-1]:g} s")
        head = f"max |{b}-{a}|"
        lines.append(f"{'channel':<24}{head:>12}{'at [s]':>9}{a + ' final':>13}{b + ' final':>13}")
        for channel, (va, vb, diff) in zip(channels, rows):
            worst = int(np.nanargmax(np.abs(diff))) if np.isfinite(diff).any() else 0
            last_a = va[np.isfinite(va)][-1] if np.isfinite(va).any() else np.nan
            last_b = vb[np.isfinite(vb)][-1] if np.isfinite(vb).any() else np.nan
            lines.append(f"{channel:<24}{abs(diff[worst]):>12.4g}{t[worst]:>9.3f}{last_a:>13.5g}{last_b:>13.5g}")
        blocks.append("\n".join(lines))
    return "\n\n".join(blocks)


def figure(runs, channels=None, labels=None, fig=None, describe=None) -> Figure:
    """Build the overlay without showing or saving it. Draws into fig when given (an empty figure, such as one
    from pyplot for a window), else into a new Figure. The first run is dashed, the others solid; the small
    plot under each channel is each later run minus the first. describe: see _prepare."""
    runs, channels, labels = _prepare(runs, channels, labels, describe)
    first = runs[0]
    count = len(runs)
    colours = [COLOURS[i % len(COLOURS)] for i in range(count)]
    rows = len(channels)
    size = (15, 2.6 * rows + 1)
    if fig is None:
        fig = Figure(figsize=size)
    else:
        fig.set_size_inches(*size)
    grid = fig.add_gridspec(2 * rows, 2, width_ratios=[1, 2.2], height_ratios=[3, 1] * rows, hspace=0.15)

    has_path = all(run.has("pos.x") and run.has("pos.y") for run in runs)
    if has_path:
        ax_path = fig.add_subplot(grid[:, 0])
        for i, run in enumerate(runs):
            x, y = run.column("pos.x"), run.column("pos.y")
            ax_path.plot(x, y, "--" if i == 0 else "-", color=colours[i], label=labels[i])
        for i, run in enumerate(runs):
            ax_path.plot(run.column("pos.x")[-1], run.column("pos.y")[-1], "o", color=colours[i])
        ax_path.set_aspect("equal", adjustable="datalim")
        ax_path.set_title("path (dot = end)")
        ax_path.set_xlabel("x [m]")
        ax_path.set_ylabel("y [m]")
        ax_path.grid(True, alpha=0.4)
        ax_path.legend(fontsize=8)

    pairs = [_pair(first, other, channels) for other in runs[1:]]
    top = None
    ax_diff = None
    for i, channel in enumerate(channels):
        ax = fig.add_subplot(grid[2 * i, 1 if has_path else slice(None)], sharex=top)
        ax_diff = fig.add_subplot(grid[2 * i + 1, 1 if has_path else slice(None)], sharex=ax)
        top = top or ax
        t0, rows0 = pairs[0]
        ax.plot(t0, rows0[i][0], "--", color=colours[0], lw=1.2, label=labels[0])
        for index, (t, values) in enumerate(pairs, start=1):
            ax.plot(t, values[i][1], "-", color=colours[index], lw=1.2, label=labels[index])
            ax_diff.plot(t, values[i][2], color="black" if count == 2 else colours[index], lw=1)
        ax_diff.axhline(0, color="grey", lw=0.6)
        ax.set_ylabel(channel)
        ax.grid(True, alpha=0.4)
        ax.tick_params(labelbottom=False)
        if i == 0:
            ax.legend(fontsize=8, loc="best")
        ax_diff.set_ylabel("B-A" if count == 2 else "run - A", fontsize=8)
        ax_diff.grid(True, alpha=0.4)
        ax_diff.tick_params(labelbottom=(i == rows - 1), labelsize=8)
    ax_diff.set_xlabel("time [s]")
    names = [f"{labels[0]} (A, dashed)"] + [f"{labels[i]} ({_letter(i)}, solid)" for i in range(1, count)]
    fig.suptitle(f"{runs[-1].name}: " + (" vs " if count == 2 else ", ").join(names))
    fig.subplots_adjust(left=0.06, right=0.98, top=0.95, bottom=0.05, wspace=0.18)
    return fig
