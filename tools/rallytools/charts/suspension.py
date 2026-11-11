"""Suspension charts for four-wheel telemetry runs."""
from __future__ import annotations

import numpy as np
from matplotlib.figure import Figure

from ..channels import derived
from ..run import Run, WHEELS
from ..suspension import window


def _shade(ax, run: Run, start, end) -> None:
    selected = window(run, start, end)
    if selected.any():
        time = run.column("time")
        ax.axvspan(float(time[selected][0]), float(time[selected][-1]), color="0.85", zorder=0)


def figure(runs, start=None, end=None) -> Figure:
    """Build the twelve-panel suspension chart without showing or saving it."""
    runs = list(runs)
    fig = Figure(figsize=(15, 10))
    grid = fig.add_gridspec(8, 4, width_ratios=(1.35, 1.35, 1, 1), hspace=0.8, wspace=0.45)

    left = [
        fig.add_subplot(grid[0:2, 0:2]),
        fig.add_subplot(grid[2:4, 0:2]),
        fig.add_subplot(grid[4:6, 0:2]),
        fig.add_subplot(grid[6:8, 0:2]),
    ]
    wheel_axes = {}
    for row, title in ((0, "Fz {wheel} (N)"), (4, "strut deflection {wheel} (m)")):
        for offset, wheel in enumerate(("FL", "FR", "RL", "RR")):
            axis_row = row + (2 if offset >= 2 else 0)
            axis = fig.add_subplot(grid[axis_row:axis_row + 2, 2 + offset % 2])
            axis.set_title(title.format(wheel=wheel))
            wheel_axes[wheel, row] = axis

    axes = left + [wheel_axes[wheel, row] for row in (0, 4) for wheel in WHEELS]
    colours = [f"C{index}" for index in range(len(runs))]
    for run, colour in zip(runs, colours):
        time = run.column("time")
        roll = np.degrees(run.column("roll"))
        a_lat = run.column("a_lat")
        front = derived(run, "transfer.front")
        rear = derived(run, "transfer.rear")
        share = derived(run, "transfer.front_share")

        left[0].plot(time, roll, color=colour, label=run.name)
        left[1].plot(a_lat, roll, color=colour)
        left[2].plot(time, front, color=colour)
        left[2].plot(time, rear, color=colour, linestyle="--")
        left[3].plot(time, share, color=colour)
        for axis in (left[0], left[2], left[3]):
            _shade(axis, run, start, end)

        for wheel in WHEELS:
            wheel_axes[wheel, 0].plot(time, run.column(f"wheel.{wheel}.Fz"), color=colour)
            wheel_axes[wheel, 4].plot(
                time, run.column(f"wheel.{wheel}.deflection"), color=colour
            )

    titles = (
        "roll (deg) over time",
        "roll (deg) against a_lat (m/s^2)",
        "lateral load transfer (N): front solid, rear dashed",
        "front share of the transfer",
    )
    for axis, title in zip(left, titles):
        axis.set_title(title)
        axis.grid(True)
    left[0].set_xlabel("time (s)")
    left[0].set_ylabel("roll (deg)")
    left[1].set_xlabel("a_lat (m/s^2)")
    left[1].set_ylabel("roll (deg)")
    left[2].set_xlabel("time (s)")
    left[2].set_ylabel("transfer (N)")
    left[3].set_xlabel("time (s)")
    left[3].set_ylabel("front share")
    left[0].legend()

    for axis in axes[4:]:
        axis.set_xlabel("time (s)")
        axis.grid(True)
    fig.subplots_adjust(left=0.06, right=0.98, bottom=0.06, top=0.96, hspace=0.8, wspace=0.45)
    return fig
