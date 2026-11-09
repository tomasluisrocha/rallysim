"""Run every scenario and make every picture, into the out/ layout (see docs/tools.md).

Usage:
  py tools/run_all.py                         every scenario + the tyre rig, then the plots (no GIFs)
  py tools/run_all.py --gif                   also the GIFs: every dashboard, and every four-wheel run in 3D (slow)
  py tools/run_all.py --only fw_brake fw_launch
  py tools/run_all.py --snapshot linear       afterwards, copy out/runs and out/rig to out/snapshots/linear
  py tools/run_all.py --no-plots              just the CSVs

Steps: rallysim <scenario> for every name `rallysim list` gives (CSVs to out/runs, the rig to out/rig), then in
parallel: the rig plot (out/plots/tyres/tyre_rig.png), a static top-down view of each run (out/plots/views), each
four-wheel run's tyres on their curve (out/plots/tyres), and with --gif the dashboards (out/gif/<run>.gif) and
the four-wheel runs in 3D (out/gif/<run>_3d.gif).
Build first (cmake --build build --config Release); this only runs what is built.
"""
import argparse
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import simdata

TOOLS = Path(__file__).resolve().parent


def find_exe(given):
    if given:
        return Path(given)
    for candidate in ("build/Release/rallysim.exe", "build/rallysim.exe", "build/Release/rallysim", "build/rallysim",
                      "build/Debug/rallysim.exe", "build/Debug/rallysim"):
        path = simdata.ROOT / candidate
        if path.exists():
            return path
    sys.exit("rallysim not found under build/; build it first or pass --exe")


def run(command):
    """Run one command from the sim folder; (ok, text) with its output."""
    done = subprocess.run([str(c) for c in command], cwd=simdata.ROOT, capture_output=True, text=True)
    return done.returncode == 0, (done.stdout + done.stderr).strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="+", metavar="NAME", help="just these scenarios")
    ap.add_argument("--gif", action="store_true", help="also write the GIFs: the dashboards and the 3D views")
    ap.add_argument("--no-plots", action="store_true", help="only write the CSVs")
    ap.add_argument("--snapshot", metavar="NAME", help="then copy out/runs and out/rig to out/snapshots/NAME")
    ap.add_argument("--exe", help="the rallysim executable (default: found under build/)")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1), help="plots made at once")
    args = ap.parse_args()

    exe = find_exe(args.exe)
    ok, listing = run([exe, "list"])
    if not ok:
        sys.exit(f"{exe} list failed (an older build without 'list'? rebuild):\n{listing}")
    names = listing.split()
    if args.only:
        unknown = [n for n in args.only if n not in names]
        if unknown:
            sys.exit(f"unknown scenario(s): {', '.join(unknown)}; there are: {', '.join(names)}")
        names = args.only

    start = time.time()
    failed = []
    print(f"running {len(names)} scenario(s) with {exe.relative_to(simdata.ROOT) if exe.is_relative_to(simdata.ROOT) else exe}")
    for name in names:
        ok, text = run([exe, name])
        print(("  " if ok else "  FAILED ") + text.splitlines()[-1] if text else name)
        if not ok:
            failed.append(name)

    if not args.no_plots:
        jobs = []
        py = sys.executable
        for name in names:
            if name == "tyre_rig":
                jobs.append((f"{name} plot", [py, TOOLS / "tyre_plot.py", simdata.RIG / "tyre_rig.csv",
                                              "--save", simdata.TYRES / "tyre_rig.png"]))
                continue
            csv = simdata.RUNS / f"{name}.csv"
            if not csv.exists():
                continue
            jobs.append((f"{name} view", [py, TOOLS / "topdown.py", csv, "--static", "--save", simdata.VIEWS / f"{name}.png"]))
            with csv.open(encoding="utf-8") as f:
                f.readline()
                header = f.readline()
            four_wheel = "wheel.FL.slipRatio" in header
            if four_wheel:
                jobs.append((f"{name} tyres", [py, TOOLS / "tyre_points.py", csv, "--save", simdata.TYRES / f"{name}.png"]))
            if args.gif:
                jobs.append((f"{name} gif", [py, TOOLS / "topdown.py", csv, "--gif", simdata.GIFS / f"{name}.gif"]))
                if four_wheel:  # view3d reads only four-wheel runs
                    jobs.append((f"{name} 3d gif", [py, TOOLS / "view3d.py", csv, "--gif", simdata.GIFS / f"{name}_3d.gif"]))
        for folder in (simdata.VIEWS, simdata.TYRES, simdata.GIFS if args.gif else None):
            if folder:
                folder.mkdir(parents=True, exist_ok=True)

        print(f"making {len(jobs)} picture(s), {args.jobs} at a time")
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            for (label, _), (ok, text) in zip(jobs, pool.map(lambda job: run(job[1]), jobs)):
                if ok:
                    print(f"  {label}")
                else:
                    print(f"  FAILED {label}:\n    " + text.replace("\n", "\n    "))
                    failed.append(label)

    if args.snapshot:
        target = simdata.SNAPSHOTS / args.snapshot
        for folder in (simdata.RUNS, simdata.RIG):
            if folder.exists():
                shutil.copytree(folder, target / folder.name, dirs_exist_ok=True)
        print(f"snapshot: out/runs and out/rig copied to {target.relative_to(simdata.ROOT)}")

    print(f"done in {time.time() - start:.0f} s" + (f"; {len(failed)} failed: {', '.join(failed)}" if failed else ""))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
