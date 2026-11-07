"""Plot columns from a telemetry CSV.

Usage:
    python tools/plot_run.py out/st_circle.csv time pos.z vel.z
    python tools/plot_run.py out/st_circle.csv time pos.z --save out/pos_z.png

The first column named after the file is the x axis; every column after it is plotted
against it. Column names come from the second line of the file (see docs/telemetry_schema.md).
"""
import argparse
import sys

import matplotlib.pyplot as plt
import pandas as pd


def main():
    parser = argparse.ArgumentParser(description="Plot columns from a telemetry CSV.")
    parser.add_argument("file", help="telemetry CSV file, e.g. out/st_circle.csv")
    parser.add_argument("x", help="column for the x axis, e.g. time")
    parser.add_argument("y", nargs="+", help="one or more columns to plot, e.g. pos.z vel.z")
    parser.add_argument("--save", metavar="PNG", help="save the figure to this file instead of showing it")
    args = parser.parse_args()

    # Lines starting with '#' (the schema/units line) are ignored.
    data = pd.read_csv(args.file, comment="#")

    wanted = [args.x] + args.y
    missing = [name for name in wanted if name not in data.columns]
    if missing:
        print("Unknown column(s): " + ", ".join(missing))
        print("Available columns: " + ", ".join(data.columns))
        sys.exit(1)

    fig, ax = plt.subplots(figsize=(9, 5))
    for name in args.y:
        ax.plot(data[args.x], data[name], label=name)
    ax.set_xlabel(args.x)
    ax.legend()
    ax.grid(True)
    ax.set_title(args.file)

    if args.save:
        fig.savefig(args.save, dpi=120, bbox_inches="tight")
        print("saved " + args.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()