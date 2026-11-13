"""Chart of chosen columns of one run against another."""
from __future__ import annotations

import numpy as np
from matplotlib.figure import Figure

from ..run import Run


def _values(run: Run, name: str, wrapped: bool) -> np.ndarray:
    values = run.column(name)
    return np.unwrap(values) if name == "yaw" and not wrapped else values


def figure(run: Run, x: str, ys, wrapped=False, fig=None) -> Figure:
    """Build the chart without showing or saving it: each y against x. Draws into fig when given
    (an empty figure, such as one from pyplot for a window), else into a new Figure. Yaw is drawn
    unwrapped unless wrapped."""
    ys = list(ys)
    missing = [name for name in [x] + ys if not run.has(name)]
    if missing:
        raise ValueError(f"run '{run.name}': unknown column(s): {', '.join(missing)}; "
                         f"available columns: {', '.join(run.data.columns)}")
    if fig is None:
        fig = Figure(figsize=(9, 5))
    else:
        fig.set_size_inches(9, 5)
    ax = fig.add_subplot()
    horizontal = _values(run, x, wrapped)
    for name in ys:
        ax.plot(horizontal, _values(run, name, wrapped), label=name)
    ax.set_xlabel(x)
    ax.set_title(run.name)
    ax.grid(True)
    ax.legend()
    return fig
