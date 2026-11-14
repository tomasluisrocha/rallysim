"""Command-line interface for numerical telemetry comparisons."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .compare import Tolerance, diff
from .paths import COMPARE, OUT, RIG, ROOT, TYRES, find_exe
from .handling import understeer
from .landing import text as landing_text
from .run import load
from .setup_sheet import text as setup_text
from .sweep import DEFAULT_METRICS, sweep, values_of
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


def _path(args):
    from .charts.path import figure
    run = load(args.run)
    if args.save is None:
        # The window needs a figure pyplot made: matplotlib 3.10 cannot show a bare Figure.
        import matplotlib.pyplot as plt
        figure(run, args.every, fig=plt.figure())
        plt.show()
        return 0
    import matplotlib
    matplotlib.use("Agg")
    chart = figure(run, args.every)
    output = Path(args.save)
    output.parent.mkdir(parents=True, exist_ok=True)
    chart.savefig(output, dpi=110, bbox_inches="tight")
    print(f"wrote {output}")
    return 0


def _characteristics(args):
    from .charts.characteristics import figure
    run = load(args.run)
    if args.save is not None and args.show:
        print("characteristics: --save and --show are mutually exclusive", file=sys.stderr)
        return 2
    if args.show:
        # The window needs a figure pyplot made: matplotlib 3.10 cannot show a bare Figure.
        import matplotlib.pyplot as plt
        figure(run, fig=plt.figure())
        plt.show()
        return 0
    import matplotlib
    matplotlib.use("Agg")
    chart = figure(run)
    if args.save is None:
        print("characteristics: give --save PNG or --show", file=sys.stderr)
        return 2
    output = Path(args.save)
    output.parent.mkdir(parents=True, exist_ok=True)
    chart.savefig(output, dpi=110)
    print(f"wrote {output}")
    return 0


def _columns(args):
    from .charts.columns import figure
    run = load(args.file)
    if args.save is not None:
        import matplotlib
        matplotlib.use("Agg")
        chart = figure(run, args.x, args.y, args.wrapped)
        output = Path(args.save)
        output.parent.mkdir(parents=True, exist_ok=True)
        chart.savefig(output, dpi=120, bbox_inches="tight")
        print(f"wrote {output}")
        return 0
    # The window needs a figure pyplot made: matplotlib 3.10 cannot show a bare Figure.
    import matplotlib.pyplot as plt
    figure(run, args.x, args.y, args.wrapped, fig=plt.figure())
    plt.show()
    return 0


def _overlay(args):
    from .charts.overlay import figure, table
    runs = [load(path) for path in args.runs]
    print(table(runs, args.channels, args.labels, paths=args.runs))
    if args.save is None:
        # The window needs a figure pyplot made: matplotlib 3.10 cannot show a bare Figure.
        import matplotlib.pyplot as plt
        figure(runs, args.channels, args.labels, fig=plt.figure())
        plt.show()
        return 0
    import matplotlib
    matplotlib.use("Agg")
    chart = figure(runs, args.channels, args.labels)
    if args.save:
        output = Path(args.save)
    else:
        names = args.labels or [run.name for run in runs]
        output = COMPARE / f"{runs[-1].name}__{'_vs_'.join(names)}.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    chart.savefig(output, dpi=100)
    print(f"wrote {output}")
    return 0


def _shown(path):
    """A path as printed: relative to the sim folder when it is inside it, else as it is."""
    path = Path(path).resolve()
    try:
        return path.relative_to(ROOT)
    except ValueError:
        return path


def _sweep(args):
    from .charts.sweep import figure
    values = values_of(args.values, args.range, args.unit)
    exe = find_exe(args.exe)
    print(f"sweeping {args.setting} over {len(values)} value(s) in {args.scenario}")
    result = sweep(exe, args.scenario, args.setting, values, args.set, args.metric, OUT, args.jobs)
    for value, text in result.failed:
        print(f"  {value}: FAILED\n    " + text.replace("\n", "\n    "))
    import pandas as pd
    with pd.option_context("display.width", 160, "display.float_format", "{:.4g}".format):
        print(result.table.to_string(index=False))
    print(f"wrote {len(result.rows)} run(s) and summary.csv to {_shown(result.folder)}")
    if not args.show:
        import matplotlib
        matplotlib.use("Agg")
    output = Path(args.save) if args.save else OUT / "plots" / "sweeps" / f"{result.folder.name}.png"
    if args.show:
        # The window needs a figure pyplot made: matplotlib 3.10 cannot show a bare Figure.
        import matplotlib.pyplot as plt
        chart = figure(result, args.trace, fig=plt.figure())
    else:
        chart = figure(result, args.trace)
    output.parent.mkdir(parents=True, exist_ok=True)
    chart.savefig(output, dpi=100)
    print(f"wrote {_shown(output)}")
    if args.show:
        plt.show()
    return 0


def _run_all(args):
    import os
    import time
    from .run_all import run_all
    exe = find_exe(args.exe)
    start = time.time()
    failed = run_all(exe, args.only, args.gif, not args.no_plots, args.snapshot, OUT,
                     args.jobs or max(1, (os.cpu_count() or 2) - 1), progress=print)
    print(f"done in {time.time() - start:.0f} s" + (f"; {len(failed)} failed: {', '.join(failed)}" if failed else ""))
    return 1 if failed else 0


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
    command = subparsers.add_parser("handling", help="understeer gradient and handling diagram of four-wheel runs",
                                    description="Print the understeer gradient of each run (the slope of the "
                                                "steer beyond the low-speed angle against a_lat), and optionally "
                                                "draw the handling diagram.")
    command.add_argument("runs", nargs="+", metavar="RUN", help="four-wheel run (CSV)")
    command.add_argument("--from", dest="start", type=float, metavar="S",
                         help="start of the window in seconds (default: the start of the run)")
    command.add_argument("--to", dest="end", type=float, metavar="S",
                         help="end of the window in seconds (default: the end of the run)")
    command.add_argument("--save", metavar="PNG", help="save a PNG headlessly (mutually exclusive with --show)")
    command.add_argument("--show", action="store_true", help="display the chart interactively (mutually exclusive with --save)")
    command = subparsers.add_parser("energy", help="the energy books of a four-wheel run: stored, work, balance",
                                    description="Print the end values of the energy books of a four-wheel run, "
                                                "and optionally draw the energy flow chart.")
    command.add_argument("run", metavar="RUN", help="four-wheel run (CSV)")
    command.add_argument("--save", metavar="PNG", help="save a PNG headlessly (mutually exclusive with --show)")
    command.add_argument("--show", action="store_true", help="display the chart interactively (mutually exclusive with --save)")
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
    command = subparsers.add_parser("characteristics", help="each suspension force against its travel or speed",
                                    description="Draw spring force and stop force against deflection and damper "
                                                "force against deflection speed, one point per row and wheel, "
                                                "from a four-wheel run's own columns.")
    command.add_argument("run", metavar="RUN", help="four-wheel run (CSV)")
    command.add_argument("--save", metavar="PNG", help="save a PNG headlessly (mutually exclusive with --show)")
    command.add_argument("--show", action="store_true", help="display the chart interactively (mutually exclusive with --save)")
    command = subparsers.add_parser("columns", help="chart of chosen columns of a run against one column",
                                    description="Plot each Y column against X. Yaw is drawn unwrapped unless "
                                                "--wrapped. Without --save the chart is shown.")
    command.add_argument("file", metavar="FILE", help="telemetry CSV file")
    command.add_argument("x", metavar="X", help="column for the x axis, e.g. time")
    command.add_argument("y", nargs="+", metavar="Y", help="one or more columns to plot")
    command.add_argument("--save", metavar="PNG", help="save the figure to this file instead of showing it")
    command.add_argument("--wrapped", action="store_true", help="draw yaw as written, between -pi and pi")
    command = subparsers.add_parser("path", help="the path of a run coloured by speed, with the car drawn along it",
                                    description="Draw the path of a run of any kind, coloured by speed, with the "
                                                "car drawn at intervals. Without --save the figure is shown.")
    command.add_argument("run", metavar="RUN", help="run (CSV)")
    command.add_argument("--every", type=float, default=0, metavar="S",
                         help="seconds between cars (default: about ten cars)")
    command.add_argument("--save", metavar="PNG", help="save the figure to this file instead of showing it")
    command = subparsers.add_parser("overlay", help="overlay of two or more runs and each minus the first",
                                    description="Draw the paths and chosen channels of the runs, and under each "
                                                "channel every later run minus the first. Prints the largest "
                                                "differences. Without --save the figure is shown.")
    command.add_argument("runs", nargs="+", metavar="RUN", help="run (CSV), at least two")
    command.add_argument("--channels", nargs="+", metavar="COL", help="channels to plot (default: by kind of run)")
    command.add_argument("--labels", nargs="+", metavar="NAME", help="a name for each run")
    command.add_argument("--save", nargs="?", const="", metavar="PNG",
                         help="write the figure (default name: out/plots/compare/<run>__<A>_vs_<B>.png)")
    command = subparsers.add_parser("sweep", help="run a scenario over a range of one setting and plot the results",
                                    formatter_class=argparse.RawDescriptionHelpFormatter,
                                    description="Run a scenario once per value of one setting "
                                                "(rallysim <scenario> --set <setting>=<value>; a car parameter "
                                                "or section.key, values may carry units) and plot how the "
                                                "results change. A metric is STAT:COLUMN worked out from each "
                                                "run: max, min, maxabs, mean, final, range. Angles are shown in "
                                                "degrees. Writes the runs and summary.csv to "
                                                "out/sweeps/<scenario>__<setting>/ and the picture to "
                                                "out/plots/sweeps/.",
                                    epilog="examples:\n"
                                           "  python tools/sweep.py fw_hairpin max_handbrake_torque 1000 2000 3000\n"
                                           "  python tools/sweep.py fw_circle spring_front --range 30 80 6 "
                                           "--unit kN/m --trace roll\n"
                                           "  python tools/sweep.py fw_brake mu_front --range 0.5 1.1 7 "
                                           "--metric final:pos.x maxabs:pitch")
    command.add_argument("scenario", help="a scenario name (rallysim list) or an .ini file")
    command.add_argument("setting", help="what to change: a car parameter, or section.key")
    command.add_argument("values", nargs="*", help="the values, as --set takes them (units allowed)")
    command.add_argument("--range", nargs=3, metavar=("FROM", "TO", "N"), help="N evenly spaced values instead")
    command.add_argument("--unit", default="", help="a unit written after every --range value (km/h, kN/m, ...)")
    command.add_argument("--metric", nargs="+", default=list(DEFAULT_METRICS), metavar="STAT:COLUMN",
                         help=f"what to measure in each run (default: {' '.join(DEFAULT_METRICS)})")
    command.add_argument("--trace", metavar="COLUMN", default="yaw_rate",
                         help="column drawn over time for every value")
    command.add_argument("--set", action="append", default=[], metavar="NAME=VALUE", help="held for every run")
    command.add_argument("--save", metavar="PNG", help="where the picture goes (default: out/plots/sweeps/...)")
    command.add_argument("--show", action="store_true", help="open the picture instead of only saving it")
    command.add_argument("--exe", help="the rallysim executable (default: found under build/)")
    command.add_argument("--jobs", type=int, default=None, metavar="N", help="runs at once (default: CPUs - 1)")
    command = subparsers.add_parser("run-all", help="run every scenario and make every picture into out/",
                                    description="Run rallysim for every scenario and the tyre rig (CSVs to "
                                                "out/runs and out/rig), then draw the path of each run, the "
                                                "tyres of each four-wheel run and the rig plot into out/plots. "
                                                "Exit 1 when anything failed.")
    command.add_argument("--only", nargs="+", metavar="NAME", help="just these scenarios")
    command.add_argument("--gif", action="store_true", help="also write the GIFs: the dashboards and the 3D views")
    command.add_argument("--no-plots", action="store_true", help="only write the CSVs")
    command.add_argument("--snapshot", metavar="NAME", help="then copy out/runs and out/rig to out/snapshots/NAME")
    command.add_argument("--exe", help="the rallysim executable (default: found under build/)")
    command.add_argument("--jobs", type=int, default=None, metavar="N", help="GIFs made at once (default: CPUs - 1)")
    args = parser.parse_args(argv)
    try:
        if args.command == "run-all":
            return _run_all(args)
        if args.command == "overlay":
            if len(args.runs) < 2:
                parser.error("overlay needs at least two runs")
            return _overlay(args)
        if args.command == "sweep":
            return _sweep(args)
        if args.command == "columns":
            return _columns(args)
        if args.command == "characteristics":
            return _characteristics(args)
        if args.command == "path":
            return _path(args)
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
        if args.command == "energy" and args.save is not None and args.show:
            print("energy: --save and --show are mutually exclusive", file=sys.stderr)
            return 2
        if args.command == "energy":
            from .energy import text as energy_text
            run = load(args.run)
            print(energy_text(run))
            if args.save is not None or args.show:
                from .charts.energy import figure
                if args.save is not None:
                    import matplotlib
                    matplotlib.use("Agg")
                    chart = figure(run)
                    output = Path(args.save)
                    output.parent.mkdir(parents=True, exist_ok=True)
                    chart.savefig(output, dpi=110)
                    print(f"wrote {output}")
                else:
                    # The window needs a figure pyplot made: matplotlib 3.10 cannot show a bare Figure.
                    import matplotlib.pyplot as plt
                    figure(run, fig=plt.figure())
                    plt.show()
            return 0
        if args.command == "handling" and args.save is not None and args.show:
            print("handling: --save and --show are mutually exclusive", file=sys.stderr)
            return 2
        if args.command == "handling":
            runs = [load(path) for path in args.runs]
            for run in runs:
                result = understeer(run, args.start, args.end)
                print(f"{run.name}: understeer gradient {result['gradient']:.6g} rad per m/s^2 "
                      f"({result['gradient_deg']:.6g} deg per m/s^2)")
            if args.save is not None or args.show:
                if args.save is not None:
                    import matplotlib
                    matplotlib.use("Agg")
                from .charts.handling import figure
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
