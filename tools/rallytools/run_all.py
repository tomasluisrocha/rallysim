"""Run every scenario and make its pictures, into the out/ layout (see docs/tools.md)."""
from __future__ import annotations

import os
import shutil
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from . import paths
from .run import load

TOOLS = paths.ROOT / "tools"


def _gif_jobs(name, csv, four_wheel, gifs):
    py = sys.executable
    jobs = [(f"{name} gif", [py, TOOLS / "topdown.py", csv, "--gif", gifs / f"{name}.gif"])]
    if four_wheel:  # view3d reads only four-wheel runs
        jobs.append((f"{name} 3d gif", [py, TOOLS / "view3d.py", csv, "--gif", gifs / f"{name}_3d.gif"]))
    return jobs


def _save(fig, output, **options):
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        fig.savefig(output, dpi=110, **options)
    finally:
        fig.clear()


def run_all(exe, only=None, gif=False, plots=True, snapshot=None, out=paths.OUT, jobs=None, progress=None):
    """Run the scenarios (`only`, default all of `rallysim list`) and the tyre rig into out/runs and out/rig,
    then draw the path of each run, the tyres of each four-wheel run and the rig plot into out/plots, in this
    process, one after another. With `gif`, also the GIFs (separate processes, `jobs` at once). With
    `snapshot`, copy out/runs and out/rig to out/snapshots/<snapshot>. `progress` is called with each line of
    progress. Returns the list of failures (empty when all went well); raises ValueError for an unknown
    scenario name or a rallysim that cannot list its scenarios."""
    say = progress or (lambda line: None)
    exe, out = Path(exe), Path(out)
    jobs = jobs or max(1, (os.cpu_count() or 2) - 1)
    ok, listing = paths.run_command([exe, "list"])
    if not ok:
        raise ValueError(f"{exe} list failed (an older build without 'list'? rebuild):\n{listing}")
    names = listing.split()
    if only:
        unknown = [n for n in only if n not in names]
        if unknown:
            raise ValueError(f"unknown scenario(s): {', '.join(unknown)}; there are: {', '.join(names)}")
        names = list(dict.fromkeys(only))

    runs, rig = out / "runs", out / "rig"
    views, tyres, gifs = out / "plots" / "views", out / "plots" / "tyres", out / "gif"
    failed = []
    say(f"running {len(names)} scenario(s) with {exe}")
    for name in names:
        target = (rig / "tyre_rig.csv") if name == "tyre_rig" else (runs / f"{name}.csv")
        target.parent.mkdir(parents=True, exist_ok=True)
        ok, text = paths.run_command([exe, name, target])
        if ok:
            say("  " + (text.splitlines()[-1] if text else name))
        else:
            say(f"  FAILED {name}: " + (text.splitlines()[-1] if text else "no message"))
            failed.append(name)

    if plots:
        import matplotlib
        matplotlib.use("Agg")
        from .charts import path as path_chart, tyre_points, tyre_rig
        todo, gif_jobs = [], []
        for name in names:
            if name == "tyre_rig":
                if (rig / "tyre_rig.csv").exists():
                    todo.append((f"{name} plot", lambda: _save(tyre_rig.figure(rig / "tyre_rig.csv"),
                                                              tyres / "tyre_rig.png")))
                continue
            csv = runs / f"{name}.csv"
            if not csv.exists():
                continue
            todo.append((f"{name} view", lambda csv=csv, name=name: _save(
                path_chart.figure(load(csv)), views / f"{name}.png", bbox_inches="tight")))
            four_wheel = load(csv).kind == "four_wheel"
            if four_wheel:
                todo.append((f"{name} tyres", lambda csv=csv, name=name: _save(
                    tyre_points.figure(load(csv)), tyres / f"{name}.png")))
            if gif:
                gif_jobs += _gif_jobs(name, csv, four_wheel, gifs)
        say(f"making {len(todo) + len(gif_jobs)} picture(s)")
        for label, draw in todo:
            try:
                draw()
                say(f"  {label}")
            except (OSError, ValueError) as error:
                say(f"  FAILED {label}:\n    " + str(error).replace("\n", "\n    "))
                failed.append(label)
        if gif_jobs:
            gifs.mkdir(parents=True, exist_ok=True)
            with ThreadPoolExecutor(max_workers=jobs) as pool:
                done = pool.map(lambda job: paths.run_command(job[1]), gif_jobs)
                for (label, _), (ok, text) in zip(gif_jobs, done):
                    if ok:
                        say(f"  {label}")
                    else:
                        say(f"  FAILED {label}:\n    " + text.replace("\n", "\n    "))
                        failed.append(label)

    if snapshot:
        target = out / "snapshots" / snapshot
        for folder in (runs, rig):
            if folder.exists():
                shutil.copytree(folder, target / folder.name, dirs_exist_ok=True)
        say(f"snapshot: runs and rig copied to {target}")
    return failed
