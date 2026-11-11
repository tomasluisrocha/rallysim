"""Text report of a run's landings. numpy only."""
from __future__ import annotations

from .events import landings
from .run import WHEELS

_NO_TRAVEL = "n/a: no rest_deflection on line 1, rerun with a newer build"


def _fmt(value, pattern, missing="n/a"):
    return missing if value is None else pattern % value


def text(run, window=2.0):
    found = landings(run, window)
    lines = [run.name]
    if not found:
        lines.append("no landing in this run")
        return "\n".join(lines)
    for n, land in enumerate(found, 1):
        lines.append(f"landing {n}: touchdown {land.t_touchdown:.4g} s after {land.airborne_time:.4g} s "
                     f"in the air (window {window:.1f} s)")
        for w in WHEELS:
            travel = _fmt(None if land.peak_travel[w] is None else land.peak_travel[w] * 1000.0,
                          "%.1f mm", _NO_TRAVEL)
            margin = _fmt(None if land.travel_margin[w] is None else land.travel_margin[w] * 1000.0, "%.1f mm")
            stop_time = _fmt(None if land.stop_time[w] is None else land.stop_time[w] * 1000.0, "%.1f")
            lines.append(f"  {w}  peak Fz {land.peak_fz[w]:.0f} N ({_fmt(land.peak_fz_nominal[w], '%.2f x fz0')})"
                         f"  travel {travel}  margin {margin}"
                         f"  stop {land.peak_stop[w]:.0f} N for {stop_time} ms")
        settling = "not settled" if land.settling_time is None else f"{land.settling_time:.3f} s"
        lines.append(f"  settling {settling}; rebounds {land.rebounds}, overshoot {land.overshoot * 100.0:.1f} %")
    return "\n".join(lines)
