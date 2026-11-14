"""Compare two runs: the paths side by side, chosen channels overlaid, and their difference underneath.

Usage:
  python tools/compare.py out/snapshots/linear/runs/fw_brake.csv out/runs/fw_brake.csv  # two files
  python tools/compare.py A.csv B.csv speed beta wheel.FL.slipRatio  # pick the channels
  python tools/compare.py --against linear fw_brake fw_launch --save  # a snapshot vs out/runs
  python tools/compare.py --against linear --all --save  # every run in both

A is drawn dashed, B solid; the small plot under each channel is B - A. B is interpolated onto A's times, over the
time both runs cover. It also prints, for each channel, the largest difference and both final values, and the
line-1 facts (car parameters) that differ between the files, so you can see what changed. Without --labels each
run is named after its origin and the parameters that differ, e.g. "snap: spring_front (50000)".
With --save the figure goes to out/plots/compare/<run>__<A>_vs_<B>.png unless a path is given.
For more than two runs use: python tools/rally.py overlay RUN RUN RUN ...
"""
import argparse
import sys
from pathlib import Path

from rallytools.charts.overlay import figure, table
from rallytools.paths import COMPARE, RUNS, SNAPSHOTS, label_of
from rallytools.run import load


def label_for(path, run):
    """'linear' for out/snapshots/linear/runs/x.csv, 'current' for out/runs/x.csv, else the parent folder."""
    path = Path(path).resolve()
    if path.parent == RUNS.resolve():
        return "current"
    label = label_of(path)
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
        base = SNAPSHOTS / args.against / "runs"
        if not base.is_dir():
            sys.exit(f"no snapshot {base}; make one with: python tools/run_all.py --snapshot {args.against}")
        names = sorted(p.stem for p in base.glob("*.csv") if (RUNS / p.name).exists()) if args.all else args.items
        if not names:
            sys.exit("name the runs to compare, or pass --all")
        pairs = [(base / f"{n}.csv", RUNS / f"{n}.csv") for n in names]
        if args.save is None and len(pairs) > 1:
            args.save = ""  # several figures: write them rather than open a window each
        channels = args.channels
    else:
        if len(args.items) < 2:
            ap.error("give two files (or --against SNAPSHOT and run names)")
        pairs = [(Path(args.items[0]), Path(args.items[1]))]
        channels = args.channels or args.items[2:]

    if args.save is not None:
        import matplotlib
        matplotlib.use("Agg")
    for path_a, path_b in pairs:
        for path in (path_a, path_b):
            if not Path(path).exists():
                sys.exit(f"missing {path}")
        run = Path(path_b).stem
        labels = tuple(args.labels) if args.labels else (label_for(path_a, run), label_for(path_b, run))
        if labels[0] == labels[1]:
            labels = ("A", "B")
        try:
            runs = [load(path_a), load(path_b)]
            describe = not args.labels
            print(table(runs, channels, labels, paths=[path_a, path_b], describe=describe))
            if args.save is None:
                import matplotlib.pyplot as plt
                figure(runs, channels, labels, fig=plt.figure(), describe=describe)
                plt.show()
                continue
            fig = figure(runs, channels, labels, describe=describe)
        except ValueError as error:
            sys.exit(str(error))
        out = Path(args.save) if (args.save and len(pairs) == 1) else COMPARE / f"{run}__{labels[0]}_vs_{labels[1]}.png"
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, dpi=100)
        print(f"wrote {out}\n")


if __name__ == "__main__":
    main()
