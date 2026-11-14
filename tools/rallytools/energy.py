"""The energy books of a four-wheel run, read from the file's own columns. numpy only."""
from __future__ import annotations

import numpy as np

from .run import Run

STORED = ("translation", "rotation", "gravity", "springs", "wheels")
WORK = ("drive", "brake", "damper", "tyre")


def _columns(run: Run):
    names = [f"energy.{n}" for n in STORED] + ["energy.total", "energy.residual"] + [f"work.{n}" for n in WORK]
    if run.kind != "four_wheel":
        raise ValueError(f"run '{run.name}': the energy books are for four-wheel runs, not {run.kind}")
    missing = [n for n in names if not run.has(n)]
    if missing:
        raise ValueError(f"run '{run.name}': no energy columns ({', '.join(missing)}); rerun with a newer build")
    return {n: run.column(n) for n in names}


def balance(run: Run) -> dict:
    """Arrays: time, the five stored terms and the four works (by column name), stored, net_work, residual and
    gap = stored - net_work - residual (zero when the file agrees with its own books)."""
    c = _columns(run)
    stored = c["energy.total"] - c["energy.total"][0]
    net_work = c["work.drive"] - c["work.brake"] - c["work.damper"] - c["work.tyre"]
    result = {"time": run.column("time")}
    result.update({n: c[n] for n in c if n != "energy.residual"})
    result.update(stored=stored, net_work=net_work, residual=c["energy.residual"],
                  gap=stored - net_work - c["energy.residual"])
    return result


def text(run: Run) -> str:
    b = balance(run)
    lines = [f"{run.name}: energy books at the end of the run (J)"]
    for n in STORED:
        lines.append(f"  energy.{n}\t{b[f'energy.{n}'][-1]:.6g}")
    for n in WORK:
        lines.append(f"  work.{n}\t{b[f'work.{n}'][-1]:.6g}")
    lines.append(f"  stored\t{b['stored'][-1]:.6g}")
    lines.append(f"  net work\t{b['net_work'][-1]:.6g}")
    lines.append(f"  residual\t{b['residual'][-1]:.6g}")
    worst = int(np.argmax(np.abs(b["residual"])))
    lines.append(f"largest residual {b['residual'][worst]:.6g} J at t = {b['time'][worst]:.6g} s")
    return "\n".join(lines)
