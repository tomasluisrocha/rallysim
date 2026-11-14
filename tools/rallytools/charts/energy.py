"""Energy flow chart for a four-wheel run."""
from __future__ import annotations

from matplotlib.figure import Figure

from ..energy import STORED, WORK, balance
from ..run import Run


def figure(run: Run, fig=None) -> Figure:
    """Build the three-panel energy chart without showing or saving it. Draws into fig when given (an empty
    figure, such as one from pyplot for a window), else into a new Figure."""
    b = balance(run)
    if fig is None:
        fig = Figure(figsize=(10, 9))
    else:
        fig.set_size_inches(10, 9)
    fig.set_layout_engine("constrained")
    stored_ax, work_ax, balance_ax = fig.subplots(3, 1, sharex=True)
    time = b["time"]
    for name in STORED:
        stored_ax.plot(time, b[f"energy.{name}"], label=name)
    for name in WORK:
        work_ax.plot(time, b[f"work.{name}"], label=name)
    balance_ax.plot(time, b["stored"], label="stored")
    balance_ax.plot(time, b["net_work"], label="net work", linestyle="--")
    balance_ax.plot(time, b["residual"], label="residual")
    for axis, title, label in ((stored_ax, f"{run.name}: stored energy", "stored energy (J)"),
                               (work_ax, "work since the start", "work since the start (J)"),
                               (balance_ax, "balance: stored against net work", "balance (J)")):
        axis.set_title(title)
        axis.set_ylabel(label)
        axis.grid(True)
        axis.legend()
    balance_ax.set_xlabel("time (s)")
    return fig
