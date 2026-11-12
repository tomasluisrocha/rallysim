"""Command-line interface for numerical telemetry comparisons."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .compare import Tolerance, diff
from .landing import text as landing_text
from .run import load
from .setup_sheet import text as setup_text
from .suspension import table as suspension_table


def _rules(values):
    result = []
    for value in values:
        pattern, separator, numbers = value.partition("=")
        if not separator or not pattern:
            raise ValueError(f"invalid tolerance {value!r}")
        parts = numbers.split(",")
        if len(parts) not in (1, 2):
            raise ValueError(f"invalid tolerance {value!r}")
        result.append((pattern, Tolerance(float(parts[0]), float(parts[1]) if len(parts) == 2 else 0.0)))
    return result


def _print_result(path_a, path_b, result, show_all):
    print(f"A: {path_a}")
    print(f"B: {path_b}")
    for key, values in result.meta_changed.items():
        print(f"meta {key}: {values[0]!r} -> {values[1]!r}")
    if result.meta_only_a:
        print("meta only A: " + ", ".join(result.meta_only_a))
    if result.meta_only_b:
        print("meta only B: " + ", ".join(result.meta_only_b))
    if result.only_a:
        print("only A: " + ", ".join(result.only_a))
    if result.only_b:
        print("only B: " + ", ".join(result.only_b))
    if result.end_a != result.end_b:
        print(f"end: A {result.end_a!r}, B {result.end_b!r}")
    columns = sorted((column for column in result.columns if column.beyond), key=lambda column: column.t_first)
    if show_all:
        columns += sorted((column for column in result.columns if not column.beyond),
                          key=lambda column: -column.largest)
    if columns:
        print("column\tlargest\tt_largest\tt_first")
    for column in columns:
        print(f"{column.name}\t{column.largest!r}\t{column.t_largest!r}\t{column.t_first!r}")


def main(argv=None):
    parser = argparse.ArgumentParser(prog="python tools/rally.py")
    subparsers = parser.add_subparsers(dest="command", required=True)
    command = subparsers.add_parser("diff", help="compare two runs column by column against a tolerance",
                                    description="Compare two runs column by column. Exit 0 within tolerance, "
                                                "1 beyond it, 2 on an error.")
    command.add_argument("a", metavar="A", help="first run (CSV)")
    command.add_argument("b", metavar="B", help="second run (CSV)")
    command.add_argument("--tol", action="append", default=[], metavar="PATTERN=ABS[,REL]",
                         help="tolerance for the columns matching PATTERN (* matches any text); "
                              "repeatable, the last matching one wins")
    command.add_argument("--ignore-missing", action="store_true",
                         help="do not count columns present in only one run as a difference")
    command.add_argument("--all", action="store_true", help="also list the columns within tolerance")
    command = subparsers.add_parser("setup", help="show the car parameters of runs side by side",
                                    description="Show every parameter on line 1 of each run side by side.")
    command.add_argument("runs", nargs="+", metavar="RUN", help="run (CSV)")
    command.add_argument("--changed", action="store_true", help="show only the parameters that differ")
    command = subparsers.add_parser("landing", help="settling, rebound and suspension travel after each landing",
                                    description="Measure each landing of a run: settling time, rebound, "
                                                "overshoot, travel and margin.")
    command.add_argument("runs", nargs="+", metavar="RUN", help="run (CSV)")
    command.add_argument("--window", type=float, default=2.0, metavar="S",
                         help="seconds after touchdown to measure over (default 2)")
    command = subparsers.add_parser("suspension", help="table and chart of body motion and suspension travel",
                                    description="Print a table of the body's motion and the suspension, "
                                                "and optionally draw the twelve-panel chart.")
    command.add_argument("runs", nargs="+", metavar="RUN", help="run (CSV)")
    command.add_argument("--from", dest="start", type=float, metavar="S",
                         help="start of the window in seconds (default: 1 s before its end)")
    command.add_argument("--to", dest="end", type=float, metavar="S",
                         help="end of the window in seconds (default: the end of the run)")
    command.add_argument("--save", metavar="PNG", help="save a PNG headlessly (mutually exclusive with --show)")
    command.add_argument("--show", action="store_true", help="display the chart interactively (mutually exclusive with --save)")
    args = parser.parse_args(argv)
    try:
        if args.command == "suspension" and args.save is not None and args.show:
            print("suspension: --save and --show are mutually exclusive", file=sys.stderr)
            return 2
        if args.command == "setup":
            runs = [load(path) for path in args.runs]
            print(setup_text(runs, args.changed))
            return 0
        if args.command == "landing":
            for path in args.runs:
                print(landing_text(load(path), args.window))
            return 0
        if args.command == "suspension":
            runs = [load(path) for path in args.runs]
            print(suspension_table(runs, args.start, args.end))
            if args.save is not None or args.show:
                if args.save is not None:
                    import matplotlib
                    matplotlib.use("Agg")
                from .charts.suspension import figure
                if args.save is not None:
                    chart = figure(runs, args.start, args.end)
                    output = Path(args.save)
                    output.parent.mkdir(parents=True, exist_ok=True)
                    chart.savefig(output, dpi=110)
                    print(f"wrote {output}")
                if args.show:
                    # The window needs a figure pyplot made: matplotlib 3.10 cannot show a bare Figure.
                    import matplotlib.pyplot as plt
                    figure(runs, args.start, args.end, fig=plt.figure())
                    plt.show()
            return 0
        rules = _rules(args.tol)
        run_a, run_b = load(args.a), load(args.b)
        result = diff(run_a, run_b, rules)
        _print_result(Path(args.a), Path(args.b), result, args.all)
        return 1 if result.differs(args.ignore_missing) else 0
    except (OSError, ValueError) as error:
        print(str(error), file=sys.stderr)
        return 2
