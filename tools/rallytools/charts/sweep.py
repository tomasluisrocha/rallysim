"""The sweep picture: each metric against the value, then one trace over time for every run."""
from __future__ import annotations

import numpy as np
from matplotlib import colormaps
from matplotlib.figure import Figure

from ..sweep import display_column, leading_number, metric_value, trailing_text


def figure(result, trace="yaw_rate", fig=None) -> Figure:
    """Build the sweep picture without showing or saving it. Draws into fig when given (an empty figure, such as
    one from pyplot for a window), else into a new Figure."""
    metrics = list(result.metrics)
    count = len(metrics)
    width = max(4.2 * count, 10)  # a sweep of one or two metrics still needs room for the trace legend
    for run in result.runs:
        run.column(trace)  # a missing column is a ValueError before anything is drawn
    if fig is None:
        fig = Figure(figsize=(width, 7.5))
    else:
        fig.set_size_inches(width, 7.5)
    grid = fig.add_gridspec(2, count)
    numbers = [leading_number(v) for v in result.values]
    numeric = all(n is not None for n in numbers)
    x = numbers if numeric else list(range(len(numbers)))
    suffixes = {trailing_text(v) for v in result.values}
    unit_text = suffixes.pop() if numeric and len(suffixes) == 1 else ""
    for k, spec in enumerate(metrics):
        ax = fig.add_subplot(grid[0, k])
        y = [row[spec] for row in result.rows]
        _, unit = metric_value(result.runs[0], spec)
        ax.plot(x, y, "o-", color="tab:blue")
        # a metric that does not change must look flat, not like noise zoomed in with an offset ("+1.7999e2")
        ax.ticklabel_format(axis="y", useOffset=False, style="plain")
        low, high = float(min(y)), float(max(y))
        middle, floor = (low + high) / 2, max(abs(low + high) / 2 * 0.05, 1e-3)
        if high - low < floor:
            ax.set_ylim(middle - floor, middle + floor)
        ax.set_title(spec, fontsize=10)
        ax.set_ylabel(unit, fontsize=9)
        ax.set_xlabel(result.setting + (f" ({unit_text})" if unit_text else ""), fontsize=9)
        if not numeric:
            ax.set_xticks(x, result.values, rotation=30, fontsize=8)
        ax.grid(True, alpha=0.3)

    axis = fig.add_subplot(grid[1, :])
    colours = colormaps["viridis"](np.linspace(0, 0.9, len(result.runs)))
    unit = ""
    for value, run, colour in zip(result.values, result.runs, colours):
        y, unit = display_column(run, trace)
        axis.plot(run.column("time"), y, color=colour, lw=1.2, label=value)
    axis.set_xlabel("time (s)")
    axis.set_ylabel(f"{trace} ({unit})" if unit else trace)
    # one column per value, about 1.6 in each, and no more than fit the axes
    axis.legend(title=result.setting, fontsize=8, ncol=max(1, min(len(result.runs), 6, int(0.85 * width / 1.6))))
    axis.grid(True, alpha=0.3)
    held_text = f"   (also {', '.join(result.held)})" if result.held else ""
    fig.suptitle(f"{result.scenario}: {result.setting} swept{held_text}", fontsize=11)
    fig.tight_layout()
    return fig
