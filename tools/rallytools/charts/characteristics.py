"""Characteristic charts of a four-wheel run: each suspension force against its travel or speed."""
from __future__ import annotations

from matplotlib.figure import Figure

from ..run import Run, WHEELS

# (x column, y column, title, x label, y label, mark rest deflection)
_PANELS = (
    ("deflection", "springF", "spring force (N) against deflection (m)", "deflection (m)", "spring force (N)", True),
    ("deflVel", "damperF", "damper force (N) against deflection speed (m/s)", "deflection speed (m/s)",
     "damper force (N)", False),
    ("deflection", "stopF", "stop force (N) against deflection (m)", "deflection (m)", "stop force (N)", True),
)


def figure(run: Run, fig=None) -> Figure:
    """Build the three characteristic panels without showing or saving them. Every point is a pair of the
    run's own columns. Draws into fig when given (an empty figure, such as one from pyplot for a window)."""
    if run.kind != "four_wheel":
        raise ValueError(f"run '{run.name}': characteristic charts need a four-wheel run, not {run.kind}")
    for x, y, *_ in _PANELS:
        for wheel in WHEELS:
            run.column(f"wheel.{wheel}.{x}")
            run.column(f"wheel.{wheel}.{y}")
    if fig is None:
        fig = Figure(figsize=(15, 5))
    else:
        fig.set_size_inches(15, 5)
    fig.set_layout_engine("constrained")
    axes = fig.subplots(1, 3)
    for axis, (x, y, title, xlabel, ylabel, rest) in zip(axes, _PANELS):
        for index, wheel in enumerate(WHEELS):
            axis.scatter(run.column(f"wheel.{wheel}.{x}"), run.column(f"wheel.{wheel}.{y}"),
                         s=4, color=f"C{index}", label=wheel)
        if rest:
            for axle, style in (("front", "-"), ("rear", "--")):
                value = run.meta.get(f"rest_deflection_{axle}")
                if value is not None:
                    axis.axvline(value, color="0.4", linewidth=0.8, linestyle=style)
        axis.set_title(title)
        axis.set_xlabel(xlabel)
        axis.set_ylabel(ylabel)
        axis.grid(True)
    axes[0].legend(markerscale=3)
    return fig
