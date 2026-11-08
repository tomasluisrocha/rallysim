"""Compare two runs: the paths side by side, chosen channels overlaid, and their difference underneath.

Usage:
  py tools/compare.py out/snapshots/linear/runs/fw_brake.csv out/runs/fw_brake.csv          two files
  py tools/compare.py A.csv B.csv speed beta wheel.FL.slipRatio                             pick the channels
  py tools/compare.py --against linear fw_brake fw_launch --save                            a snapshot vs out/runs
  py tools/compare.py --against linear --all --save                                         every run in both

A is drawn dashed, B solid; the small plot under each channel is B - A. B is interpolated onto A's times, over the
time both runs cover. It also prints, for each channel, the largest difference and both final values, and the
line-1 facts (car parameters) that differ between the files, so you can see what changed.
With --save the figure goes to out/plots/compare/<run>__<A>_vs_<B>.png unless a path is given.
"""
import argparse
import sys
from pathlib import Path

import matplotlib
import numpy as np

import simdata

DEFAULTS = {
    "four_wheel": ["speed", "yaw_rate", "beta", "a_lat", "pitch"],
    "single_track": ["speed", "yaw_rate", "beta", "a_lat"],
    "bicycle": ["speed", "yaw_rate", "beta", "a_lat"],
    "rigid_body": ["pos.z", "vel.x", "vel.z", "omega.z"],
}
ANGLES = {"yaw", "beta"}  # differences wrapped to [-pi, pi)
STILL_SPEED = 0.3          # m/s: below this beta is not compared (as in topdown.py)


def kind(data):
    if simdata.is_four_wheel(data):
        return "four_wheel"
    if "axle.front.Fy" in data.columns:
        return "single_track"
    if "accel" in data.columns:
        return "bicycle"
    return "rigid_body"


def compare(path_a, path_b, channels, labels):
    a, meta_a = simdata.read_run(path_a)
    b, meta_b = simdata.read_run(path_b)
    simdata.unwrap_headings(a)  # yaw plotted continuously, not as a sawtooth; B - A is still wrapped below
    simdata.unwrap_headings(b)
    channels = channels or DEFAULTS[kind(a)]
    missing = [c for c in channels if c not in a.columns or c not in b.columns]
    if missing:
        sys.exit(f"not in both files: {', '.join(missing)}\nin {path_a}: {', '.join(a.columns)}")

    ta, tb = a["time"].to_numpy(), b["time"].to_numpy()
    keep = (ta >= tb[0]) & (ta <= tb[-1])
    t = ta[keep]
    label_a, label_b = labels

    print(f"A = {label_a}  ({path_a})\nB = {label_b}  ({path_b})")
    changed = sorted(k for k in set(meta_a) | set(meta_b) if meta_a.get(k) != meta_b.get(k))
    if changed:
        print("line-1 facts that differ: " + ", ".join(f"{k} {meta_a.get(k)} -> {meta_b.get(k)}" for k in changed))
    else:
        print("line-1 facts: the same")
    if ta[-1] != tb[-1]:
        print(f"note: A ends at {ta[-1]:g} s, B at {tb[-1]:g} s; compared over {t[0]:g} to {t[-1]:g} s")
    print(f"{'channel':<24}{'max |B-A|':>12}{'at [s]':>9}{'A final':>13}{'B final':>13}")

    import matplotlib.pyplot as plt
    rows = len(channels)
    fig = plt.figure(figsize=(15, 2.6 * rows + 1))
    grid = fig.add_gridspec(2 * rows, 2, width_ratios=[1, 2.2], height_ratios=[3, 1] * rows, hspace=0.15)

    has_path = "pos.x" in a.columns and "pos.y" in a.columns and "pos.x" in b.columns
    if has_path:
        ax_path = fig.add_subplot(grid[:, 0])
        ax_path.plot(a["pos.x"], a["pos.y"], "--", color="tab:blue", label=label_a)
        ax_path.plot(b["pos.x"], b["pos.y"], "-", color="tab:red", label=label_b)
        ax_path.plot(a["pos.x"].iloc[-1], a["pos.y"].iloc[-1], "o", color="tab:blue")
        ax_path.plot(b["pos.x"].iloc[-1], b["pos.y"].iloc[-1], "o", color="tab:red")
        ax_path.set_aspect("equal", adjustable="datalim")
        ax_path.set_title("path (dot = end)")
        ax_path.set_xlabel("x [m]")
        ax_path.set_ylabel("y [m]")
        ax_path.grid(True, alpha=0.4)
        ax_path.legend(fontsize=8)

    first = None
    for i, channel in enumerate(channels):
        ax = fig.add_subplot(grid[2 * i, 1 if has_path else slice(None)], sharex=first)
        ax_diff = fig.add_subplot(grid[2 * i + 1, 1 if has_path else slice(None)], sharex=ax)
        first = first or ax
        va = a[channel].to_numpy(dtype=float)[keep]
        vb = np.interp(t, tb, b[channel].to_numpy(dtype=float))
        if channel == "beta" and "speed" in a.columns and "speed" in b.columns:
            # sideslip means nothing at a standstill (it flips between 0 and pi): leave those moments out
            va[a["speed"].to_numpy()[keep] < STILL_SPEED] = np.nan
            vb[np.interp(t, tb, b["speed"].to_numpy()) < STILL_SPEED] = np.nan
        diff = vb - va
        if channel in ANGLES:
            diff = (diff + np.pi) % (2 * np.pi) - np.pi  # a heading of pi and -pi is the same heading
        worst = int(np.nanargmax(np.abs(diff))) if np.isfinite(diff).any() else 0
        last_a = va[np.isfinite(va)][-1] if np.isfinite(va).any() else np.nan
        last_b = vb[np.isfinite(vb)][-1] if np.isfinite(vb).any() else np.nan
        print(f"{channel:<24}{abs(diff[worst]):>12.4g}{t[worst]:>9.3f}{last_a:>13.5g}{last_b:>13.5g}")

        ax.plot(t, va, "--", color="tab:blue", lw=1.2, label=label_a)
        ax.plot(t, vb, "-", color="tab:red", lw=1.2, label=label_b)
        ax.set_ylabel(channel)
        ax.grid(True, alpha=0.4)
        ax.tick_params(labelbottom=False)
        if i == 0:
            ax.legend(fontsize=8, loc="best")
        ax_diff.plot(t, diff, color="black", lw=1)
        ax_diff.axhline(0, color="grey", lw=0.6)
        ax_diff.set_ylabel("B-A", fontsize=8)
        ax_diff.grid(True, alpha=0.4)
        ax_diff.tick_params(labelbottom=(i == rows - 1), labelsize=8)
    ax_diff.set_xlabel("time [s]")
    fig.suptitle(f"{Path(path_b).stem}: {label_a} (A, dashed) vs {label_b} (B, solid)")
    fig.subplots_adjust(left=0.06, right=0.98, top=0.95, bottom=0.05, wspace=0.18)
    return fig


def label_for(path, run):
    """'linear' for out/snapshots/linear/runs/x.csv, 'current' for out/runs/x.csv, else the parent folder."""
    path = Path(path).resolve()
    if path.parent == simdata.RUNS.resolve():
        return "current"
    label = simdata.label_of(path)
    return label.split("/")[0] if "/" in label else path.parent.name or run


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("items", nargs="*", help="A.csv B.csv [channels...], or with --against: run names")
    ap.add_argument("--against", metavar="SNAPSHOT", help="compare out/snapshots/SNAPSHOT/runs/<run>.csv with out/runs")
    ap.add_argument("--all", action="store_true", help="with --against: every run that is in both")
    ap.add_argument("--channels", nargs="+", metavar="COL", help="channels to plot (default: by kind of run)")
    ap.add_argument("--labels", nargs=2, metavar=("A", "B"), help="names for the two runs in the plot")
    ap.add_argument("--save", nargs="?", const="", metavar="PNG",
                    help="write the figure(s); default names in out/plots/compare/")
    args = ap.parse_args()

    if args.against:
        base = simdata.SNAPSHOTS / args.against / "runs"
        if not base.is_dir():
            sys.exit(f"no snapshot {base}; make one with: py tools/run_all.py --snapshot {args.against}")
        names = sorted(p.stem for p in base.glob("*.csv") if (simdata.RUNS / p.name).exists()) if args.all else args.items
        if not names:
            sys.exit("name the runs to compare, or pass --all")
        pairs = [(base / f"{n}.csv", simdata.RUNS / f"{n}.csv") for n in names]
        if args.save is None and len(pairs) > 1:
            args.save = ""  # several figures: write them rather than open a window each
        channels = args.channels
    else:
        if len(args.items) < 2:
            ap.error("give two files (or --against SNAPSHOT and run names)")
        pairs = [(Path(args.items[0]), Path(args.items[1]))]
        channels = args.channels or args.items[2:]

    if args.save is not None:
        matplotlib.use("Agg")
    for path_a, path_b in pairs:
        for path in (path_a, path_b):
            if not Path(path).exists():
                sys.exit(f"missing {path}")
        run = Path(path_b).stem
        labels = tuple(args.labels) if args.labels else (label_for(path_a, run), label_for(path_b, run))
        if labels[0] == labels[1]:
            labels = ("A", "B")
        fig = compare(path_a, path_b, channels, labels)
        if args.save is None:
            import matplotlib.pyplot as plt
            plt.show()
            continue
        out = Path(args.save) if (args.save and len(pairs) == 1) else simdata.COMPARE / f"{run}__{labels[0]}_vs_{labels[1]}.png"
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, dpi=100)
        print(f"wrote {out}\n")
        import matplotlib.pyplot as plt
        plt.close(fig)


if __name__ == "__main__":
    main()
