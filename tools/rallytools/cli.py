"""Command-line interface for numerical telemetry comparisons."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .compare import Tolerance, diff
from .paths import RIG, TYRES
from .landing import text as landing_text
from .run import load
from .setup_sheet import text as setup_text
from .stepsize import report as stepsize_report, text as stepsize_text
from .suspension import table as suspension_table
from .tyre import check_against_rig, tyre_from_meta


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


def _tyre_points(args):
    from .charts.tyre_points import figure
    run = load(args.run)
    if run.kind == "four_wheel":
        missing = tyre_from_meta(run.meta, "front")["missing"]
        if missing:
            print(f"note: {args.run} has no tyre keys on line 1 ({', '.join(missing)}); using the defaults",
                  file=sys.stderr)
        error = check_against_rig(RIG / "tyre_rig.csv")
        if error is not None and error > 1.0:
            print(f"warning: rallytools/tyre.py differs from the C++ rig by up to {error:.1f} N; update it",
                  file=sys.stderr)
    if args.save is not None:
        import matplotlib
        matplotlib.use("Agg")
    if args.save is None:
        # The window needs a figure pyplot made: matplotlib 3.10 cannot show a bare Figure.
        import matplotlib.pyplot as plt
        figure(run, args.min_speed, fig=plt.figure())
        plt.show()
        return 0
    chart = figure(run, args.min_speed)
    output = Path(args.save) if args.save else TYRES / f"{Path(args.run).stem}.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    chart.savefig(output, dpi=110)
    print(f"wrote {output}")
    return 0


def _tyre_rig(args):
    from .charts.tyre_rig import figure
    if args.save is not None:
        import matplotlib
        matplotlib.use("Agg")
        chart = figure(args.file, args.curve, args.combined)
        output = Path(args.save)
        output.parent.mkdir(parents=True, exist_ok=True)
        chart.savefig(output, dpi=110)
        print(f"wrote {output}")
        return 0
    # The window needs a figure pyplot made: matplotlib 3.10 cannot show a bare Figure.
    import matplotlib.pyplot as plt
    figure(args.file, args.curve, args.combined, fig=plt.figure())
    plt.show()
    return 0


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
    command = subparsers.add_parser("stepsize", help="compare one scenario run at dt, dt/2 and dt/4",
                                    description="Compare runs of one scenario at dt, dt/2 and dt/4. Exit 0 when "
                                                "no column is not converging, 1 otherwise, 2 on an error.")
    command.add_argument("a", metavar="A", help="run at dt (CSV)")
    command.add_argument("b", metavar="B", help="run at dt/2 (CSV)")
    command.add_argument("c", metavar="C", help="run at dt/4 (CSV)")
    command.add_argument("--all", action="store_true", help="also list the converging and equal columns")
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
    command.add_argument("--events", action="store_true",
                         help="mark the first run's events and landings on the chart (needs --save or --show)")
    command = subparsers.add_parser("tyre-points", help="chart of a four-wheel run's tyres on their force curve",
                                    description="Draw every tyre of a four-wheel run as a point on its force "
                                                "curve. Without --save the chart is shown.")
    command.add_argument("run", metavar="RUN", help="a four-wheel run (CSV)")
    command.add_argument("--save", nargs="?", const="", metavar="PNG",
                         help="write the chart (default name: out/plots/tyres/<run>.png)")
    command.add_argument("--min-speed", type=float, default=1.0, metavar="M/S",
                         help="leave out slower moments (default 1)")
    command = subparsers.add_parser("tyre-rig", help="chart of the single-tyre rig: force against slip",
                                    description="Draw Fx against slip ratio and Fy against slip angle at each "
                                                "load, and one curve's combined sweeps. Make the CSV with "
                                                "rallysim tyre_rig. Without --save the chart is shown.")
    command.add_argument("file", nargs="?", default=str(RIG / "tyre_rig.csv"), metavar="FILE",
                         help="rig CSV (default out/rig/tyre_rig.csv)")
    command.add_argument("--curve", nargs="+", metavar="NAME",
                         help="curves for the pure-slip panels (default: all in the file)")
    command.add_argument("--combined", metavar="NAME",
                         help="curve for the combined panels (default: the first --curve, else tmeasy)")
    command.add_argument("--save", metavar="PNG", help="save the chart to this file instead of showing it")
    args = parser.parse_args(argv)
    try:
        if args.command == "tyre-rig":
            return _tyre_rig(args)
        if args.command == "tyre-points":
            return _tyre_points(args)
        if args.command == "suspension" and args.events and args.save is None and not args.show:
            print("suspension: --events needs --save or --show", file=sys.stderr)
            return 2
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
        if args.command == "stepsize":
            rows = stepsize_report(load(args.a), load(args.b), load(args.c))
            for label, path in (("A", args.a), ("B", args.b), ("C", args.c)):
                print(f"{label}: {Path(path)}")
            print(stepsize_text(rows, args.all))
            return 1 if any(row.verdict == "not converging" for row in rows) else 0
        if args.command == "suspension":
            runs = [load(path) for path in args.runs]
            print(suspension_table(runs, args.start, args.end))
            if args.save is not None or args.show:
                if args.save is not None:
                    import matplotlib
                    matplotlib.use("Agg")
                from .charts.suspension import figure
                if args.events and "wheel_radius" not in runs[0].meta:
                    print(f"suspension: run '{runs[0].name}' has no wheel_radius on line 1; no marks drawn",
                          file=sys.stderr)
                if args.save is not None:
                    chart = figure(runs, args.start, args.end, marks=args.events)
                    output = Path(args.save)
                    output.parent.mkdir(parents=True, exist_ok=True)
                    chart.savefig(output, dpi=110)
                    print(f"wrote {output}")
                if args.show:
                    # The window needs a figure pyplot made: matplotlib 3.10 cannot show a bare Figure.
                    import matplotlib.pyplot as plt
                    figure(runs, args.start, args.end, fig=plt.figure(), marks=args.events)
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
