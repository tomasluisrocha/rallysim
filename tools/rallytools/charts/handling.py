"""Handling diagram for four-wheel telemetry runs."""
from __future__ import annotations

import numpy as np
from matplotlib.figure import Figure

from ..handling import understeer


def figure(runs, start=None, end=None, fig=None) -> Figure:
    """Build the handling diagram without showing or saving it. Draws into fig when given (an empty figure,
    such as one from pyplot for a window), else into a new Figure."""
    runs = list(runs)
    if fig is None:
        fig = Figure(figsize=(11, 5))
    else:
        fig.set_size_inches(11, 5)
    fig.set_layout_engine("constrained")
    steer_axis, slip_axis = fig.subplots(1, 2)
    for index, run in enumerate(runs):
        colour = f"C{index}"
        result = understeer(run, start, end)
        a_lat = result["a_lat"]
        steer_axis.plot(a_lat, np.degrees(result["extra_steer"]), ".", markersize=3, color=colour, label=run.name)
        ends = np.array([a_lat.min(), a_lat.max()])
        fitted = result["gradient"] * ends + result["intercept"]
        steer_axis.plot(ends, np.degrees(fitted), "--", color=colour)
        slip_axis.plot(a_lat, np.degrees(result["front_slip"]), "-", color=colour, label=run.name)
        slip_axis.plot(a_lat, np.degrees(result["rear_slip"]), "--", color=colour)
    steer_axis.set_title("extra steer (deg) against a_lat (m/s^2)")
    steer_axis.set_xlabel("a_lat (m/s^2)")
    steer_axis.set_ylabel("extra steer (deg)")
    slip_axis.set_title("axle slip angles (deg) against a_lat: front solid, rear dashed")
    slip_axis.set_xlabel("a_lat (m/s^2)")
    slip_axis.set_ylabel("slip angle (deg)")
    for axis in (steer_axis, slip_axis):
        axis.grid(True)
        axis.legend()
    return fig
