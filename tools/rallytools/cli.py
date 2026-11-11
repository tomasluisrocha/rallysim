"""Command-line interface for numerical telemetry comparisons."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .compare import Tolerance, diff
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
    columns = [column for column in result.columns if column.beyond]
    if show_all:
        columns += [column for column in result.columns if not column.beyond]
        columns.sort(key=lambda column: (-column.largest if show_all else 0))
    else:
        columns.sort(key=lambda column: column.t_first)
    for column in columns:
        print(f"{column.name}\t{column.largest!r}\t{column.t_largest!r}\t{column.t_first!r}")


def main(argv=None):
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    command = subparsers.add_parser("diff")
    command.add_argument("a")
    command.add_argument("b")
    command.add_argument("--tol", action="append", default=[])
    command.add_argument("--ignore-missing", action="store_true")
    command.add_argument("--all", action="store_true")
    command = subparsers.add_parser("setup")
    command.add_argument("runs", nargs="+")
    command.add_argument("--changed", action="store_true")
    command = subparsers.add_parser("suspension")
    command.add_argument("runs", nargs="+")
    command.add_argument("--from", dest="start", type=float)
    command.add_argument("--to", dest="end", type=float)
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
        if args.command == "suspension":
            runs = [load(path) for path in args.runs]
            print(suspension_table(runs, args.start, args.end))
            if args.save is not None or args.show:
                if args.save is not None:
                    import matplotlib
                    matplotlib.use("Agg")
                from .charts.suspension import figure
                chart = figure(runs, args.start, args.end)
                if args.save is not None:
                    output = Path(args.save)
                    output.parent.mkdir(parents=True, exist_ok=True)
                    chart.savefig(output, dpi=110)
                    print(f"wrote {output}")
                if args.show:
                    import matplotlib.pyplot as plt
                    plt.figure(chart)
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
