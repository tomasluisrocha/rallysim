"""Run one scenario over a range of one setting, and plot how the run's results change with it.

Each value is a run of `rallysim <scenario> --set <setting>=<value>`; the setting is anything --set takes, a car
parameter (spring_front) or another section's key (driver.max_yaw_rate), and the values may carry units.

Usage:
  py tools/sweep.py fw_hairpin max_handbrake_torque 1000 2000 3000 4000
  py tools/sweep.py fw_hairpin max_handbrake_torque --range 1000 4000 7       7 values from 1000 to 4000
  py tools/sweep.py fw_brake mu_front --range 0.5 1.1 7 --metric final:pos.x maxabs:pitch
  py tools/sweep.py fw_circle spring_front --range 30 80 6 --unit kN/m --trace roll
  py tools/sweep.py fw_showcase driver.max_yaw_rate 40deg/s 60deg/s 80deg/s --set mu_rear=0.85

A metric is statistic:column, worked out from each run's telemetry: max, min, maxabs, mean, final, range (max -
min). Angles are shown in degrees. --trace draws one column over time for every value.

Output: the runs and a summary table (summary.csv, one row per value) in out/sweeps/<scenario>__<setting>/, and the
picture in out/plots/sweeps/<scenario>__<setting>.png.
"""
import argparse
import math
import os
import re
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd

import simdata

DEFAULT_METRICS = ("maxabs:beta", "maxabs:yaw_rate", "final:yaw", "final:speed")
STATISTICS = {
    "max": lambda v: float(np.max(v)),
    "min": lambda v: float(np.min(v)),
    "maxabs": lambda v: float(np.max(np.abs(v))),
    "mean": lambda v: float(np.mean(v)),
    "final": lambda v: float(v[-1]),
    "range": lambda v: float(np.max(v) - np.min(v)),
}


def leading_number(text):
    """The number a value starts with ("45kN/m" -> 45.0), for the plot's x axis; None if it has none."""
    match = re.match(r"\s*[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?", text)
    return float(match.group(0)) if match else None


def safe_name(text):
    """Text usable in a file name: "driver.max_yaw_rate" stays, "40deg/s" becomes "40deg_s"."""
    return re.sub(r"[^A-Za-z0-9._+-]", "_", text)


def metric_value(data, units, spec):
    """The metric `spec` ("maxabs:beta") of one run, in display units (degrees for angles), and its unit."""
    statistic, _, column = spec.partition(":")
    values = data[column].to_numpy(dtype=float)
    if column in simdata.HEADINGS:
        values = np.unwrap(values)  # a heading counts on past +-pi, so "final:yaw" is the turn made
    value = STATISTICS[statistic](values)
    unit = units.get(column, "")
    if unit in ("rad", "rad/s"):
        return math.degrees(value), unit.replace("rad", "deg")
    return value, unit


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", help="a scenario name (rallysim list) or an .ini file")
    ap.add_argument("setting", help="what to change: a car parameter, or section.key")
    ap.add_argument("values", nargs="*", help="the values, as --set takes them (units allowed)")
    ap.add_argument("--range", nargs=3, metavar=("FROM", "TO", "N"), help="N evenly spaced values instead")
    ap.add_argument("--unit", default="", help="a unit written after every --range value (km/h, kN/m, ...)")
    ap.add_argument("--metric", nargs="+", default=list(DEFAULT_METRICS), metavar="STAT:COLUMN",
                    help=f"what to measure in each run (default: {' '.join(DEFAULT_METRICS)})")
    ap.add_argument("--trace", metavar="COLUMN", default="yaw_rate", help="column drawn over time for every value")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="held for every run")
    ap.add_argument("--save", metavar="PNG", help="where the picture goes (default: out/plots/sweeps/...)")
    ap.add_argument("--show", action="store_true", help="open the picture instead of only saving it")
    ap.add_argument("--exe", help="the rallysim executable (default: found under build/)")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1), help="runs at once")
    args = ap.parse_args()

    if args.range:
        low, high, count = float(args.range[0]), float(args.range[1]), int(args.range[2])
        values = [f"{v:g}{args.unit}" for v in np.linspace(low, high, count)]
    else:
        values = args.values
    if not values:
        sys.exit("give the values, or --range FROM TO N")
    for spec in args.metric:
        statistic, sep, _ = spec.partition(":")
        if not sep or statistic not in STATISTICS:
            sys.exit(f"metric '{spec}': expected STAT:COLUMN, STAT one of {', '.join(STATISTICS)}")

    label = f"{Path(args.scenario).stem}__{safe_name(args.setting)}"
    folder = simdata.SWEEPS / label
    folder.mkdir(parents=True, exist_ok=True)
    exe = simdata.find_exe(args.exe)

    held = [arg for pair in args.set for arg in ("--set", pair)]
    jobs = []
    for k, value in enumerate(values):
        csv = folder / f"{k + 1:02d}_{safe_name(value)}.csv"
        jobs.append((value, csv, [exe, args.scenario, csv, "--set", f"{args.setting}={value}", *held]))
    print(f"sweeping {args.setting} over {len(values)} value(s) in {args.scenario}")
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        results = list(pool.map(lambda job: simdata.run_command(job[2]), jobs))

    rows, runs = [], []
    for (value, csv, _), (ok, text) in zip(jobs, results):
        if not ok:
            print(f"  {value}: FAILED\n    " + text.replace("\n", "\n    "))
            continue
        data = pd.read_csv(csv, comment="#")
        units = simdata.read_units(csv)
        missing = [spec.partition(":")[2] for spec in args.metric if spec.partition(":")[2] not in data.columns]
        if missing:
            sys.exit(f"no column {', '.join(missing)} in {csv.name}")
        row = {"value": value}
        for spec in args.metric:
            row[spec], _ = metric_value(data, units, spec)
        rows.append(row)
        runs.append((value, data, units))
    if not rows:
        sys.exit("every run failed")

    table = pd.DataFrame(rows)
    table.to_csv(folder / "summary.csv", index=False)
    with pd.option_context("display.width", 160, "display.float_format", "{:.4g}".format):
        print(table.to_string(index=False))
    print(f"wrote {len(rows)} run(s) and summary.csv to {folder.relative_to(simdata.ROOT)}")

    # ---- the picture: each metric against the value, then the trace of every run over time ----------------------
    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    numbers = [leading_number(v) for v in table["value"]]
    numeric = all(n is not None for n in numbers)
    x = numbers if numeric else list(range(len(numbers)))
    count = len(args.metric)
    fig, axes = plt.subplots(2, count, figsize=(4.2 * count, 7.5), squeeze=False)
    units0 = runs[0][2]
    for k, spec in enumerate(args.metric):
        ax = axes[0][k]
        _, unit = metric_value(runs[0][1], units0, spec)
        ax.plot(x, table[spec], "o-", color="tab:blue")
        # a metric that does not change must look flat, not like noise zoomed in with an offset ("+1.7999e2")
        ax.ticklabel_format(axis="y", useOffset=False, style="plain")
        low, high = float(table[spec].min()), float(table[spec].max())
        middle, floor = (low + high) / 2, max(abs(low + high) / 2 * 0.05, 1e-3)
        if high - low < floor:
            ax.set_ylim(middle - floor, middle + floor)
        ax.set_title(spec, fontsize=10)
        ax.set_ylabel(unit, fontsize=9)
        ax.set_xlabel(args.setting + (f" ({args.unit})" if args.unit else ""), fontsize=9)
        if not numeric:
            ax.set_xticks(x, table["value"], rotation=30, fontsize=8)
        ax.grid(True, alpha=0.3)

    trace = plt.subplot2grid((2, count), (1, 0), colspan=count, fig=fig)
    for ax in axes[1]:
        ax.remove()
    colours = plt.get_cmap("viridis")(np.linspace(0, 0.9, len(runs)))
    for (value, data, units), colour in zip(runs, colours):
        if args.trace not in data.columns:
            sys.exit(f"no column {args.trace} in the runs")
        y = data[args.trace].to_numpy(dtype=float)
        if args.trace in simdata.HEADINGS:
            y = np.unwrap(y)
        unit = units.get(args.trace, "")
        if unit in ("rad", "rad/s"):
            y = np.degrees(y)
            unit = unit.replace("rad", "deg")
        trace.plot(data["time"], y, color=colour, lw=1.2, label=value)
    trace.set_xlabel("time (s)")
    trace.set_ylabel(f"{args.trace} ({unit})" if unit else args.trace)
    trace.legend(title=args.setting, fontsize=8, ncol=min(len(runs), 6))
    trace.grid(True, alpha=0.3)
    held_text = f"   (also {', '.join(args.set)})" if args.set else ""
    fig.suptitle(f"{args.scenario}: {args.setting} swept{held_text}", fontsize=11)
    fig.tight_layout()

    save = Path(args.save) if args.save else simdata.SWEEP_PLOTS / f"{label}.png"
    save.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save, dpi=100)
    print(f"wrote {save.relative_to(simdata.ROOT) if save.resolve().is_relative_to(simdata.ROOT) else save}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
