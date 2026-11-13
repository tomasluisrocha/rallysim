"""Vertical marks for events on time charts."""
from __future__ import annotations

# None of these is one of the default cycle colours C0 to C9, which the runs use.
KINDS = {
    "lock": "#b00020",
    "spin": "#e6007e",
    "lift": "#00897b",
    "bump_stop": "#6d4c41",
    "droop_stop": "#5e35b1",
    "touchdown": "#212121",
}


def mark(axes, times, colour, label=None) -> None:
    """Draw a vertical line at each time on each of the axes; the label goes on the first line of the first axes."""
    for a, axis in enumerate(axes):
        for i, time in enumerate(times):
            axis.axvline(time, color=colour, linewidth=0.8, zorder=1,
                         label=label if (a == 0 and i == 0) else None)
