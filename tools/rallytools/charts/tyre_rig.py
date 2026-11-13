"""The single-tyre rig chart: force against slip for each tyre curve."""
from __future__ import annotations

import matplotlib
import numpy as np
import pandas as pd
from matplotlib.figure import Figure

STYLES = {"linear": "--", "brush": ":", "peak_slide": (0, (6, 2, 1, 2)), "tmeasy": "-"}


def figure(path, curves=None, combined=None, fig=None) -> Figure:
    """Build the four-panel rig chart from a rig CSV without showing or saving it. Draws into fig when given
    (an empty figure, such as one from pyplot for a window), else into a new Figure. The pure-slip panels show
    `curves` (default: all in the file); the combined panels show one curve: `combined`, else the first of
    `curves`, else tmeasy (the car's)."""
    data = pd.read_csv(path, comment="#")
    present = list(dict.fromkeys(data["curve"]))
    chosen = list(curves) if curves else present
    if combined is None:
        combined = curves[0] if curves else "tmeasy"
    unknown = [c for c in dict.fromkeys(chosen + [combined]) if c not in present]
    if unknown:
        raise ValueError(f"unknown curve(s): {', '.join(unknown)}; in the file: {', '.join(present)}")

    if fig is None:
        fig = Figure(figsize=(14, 9))
    else:
        fig.set_size_inches(14, 9)
    axes = fig.subplots(2, 2)
    ax_x, ax_y, ax_c, ax_circle = axes.ravel()

    viridis = matplotlib.colormaps["viridis"]
    plasma = matplotlib.colormaps["plasma"]
    loads = sorted(data.loc[data["sweep"] == "x", "fz"].unique())
    load_colour = {fz: viridis(i / max(len(loads) - 1, 1)) for i, fz in enumerate(loads)}

    for curve in chosen:
        for fz in loads:
            pure_x = data[(data["curve"] == curve) & (data["sweep"] == "x") & (data["fz"] == fz)]
            pure_y = data[(data["curve"] == curve) & (data["sweep"] == "y") & (data["fz"] == fz)]
            style = STYLES.get(curve, "-.")
            label = f"{curve}, Fz {fz:.0f} N"
            ax_x.plot(pure_x["slip_ratio"], pure_x["fx"], linestyle=style, color=load_colour[fz], label=label)
            ax_y.plot(pure_y["slip_angle_deg"], pure_y["fy"], linestyle=style, color=load_colour[fz], label=label)

    ax_x.set_title("Pure longitudinal slip")
    ax_x.set_xlabel("slip ratio")
    ax_x.set_ylabel("Fx [N]")
    ax_y.set_title("Pure lateral slip")
    ax_y.set_xlabel("slip angle [deg]")
    ax_y.set_ylabel("Fy [N]")
    ax_x.legend(fontsize=7, ncol=len(chosen))

    rows_c = data[(data["curve"] == combined) & (data["sweep"] == "combined")]
    angles = sorted(rows_c["slip_angle_deg"].unique())
    angle_colour = {a: plasma(i / max(len(angles) - 1, 1) * 0.85) for i, a in enumerate(angles)}
    for angle in angles:
        rows = rows_c[rows_c["slip_angle_deg"] == angle]
        ax_c.plot(rows["slip_ratio"], rows["fx"], "-", color=angle_colour[angle], label=f"Fx, {angle:g} deg")
        ax_c.plot(rows["slip_ratio"], rows["fy"], "--", color=angle_colour[angle], label=f"Fy, {angle:g} deg")
        ax_circle.plot(rows["fx"], rows["fy"], "-", color=angle_colour[angle], label=f"{angle:g} deg")
    ax_c.set_title(f"Combined slip ({combined}): the side force falls as the wheel slips")
    ax_c.set_xlabel("slip ratio")
    ax_c.set_ylabel("force [N]")
    ax_c.legend(fontsize=7, ncol=2)

    # Grip at the combined sweep's load: the linear tyre's pure-slip maximum there is exactly mu Fz.
    fz_c = rows_c["fz"].iloc[0]
    reference = data[(data["sweep"] == "x") & (data["fz"] == fz_c)]
    linear = reference[reference["curve"] == "linear"]
    grip = (linear if len(linear) else reference)["fx"].abs().max()
    theta = np.linspace(0, 2 * np.pi, 361)
    ax_circle.plot(grip * np.cos(theta), grip * np.sin(theta), color="grey", lw=1, label=f"mu Fz = {grip:.0f} N")
    ax_circle.set_title(f"Combined slip ({combined}): Fy against Fx, Fz {fz_c:.0f} N")
    ax_circle.set_xlabel("Fx [N]")
    ax_circle.set_ylabel("Fy [N]")
    ax_circle.set_aspect("equal", adjustable="datalim")
    ax_circle.legend(fontsize=7)

    for ax in axes.ravel():
        ax.grid(True, alpha=0.4)
        ax.axhline(0, color="black", lw=0.6)
        ax.axvline(0, color="black", lw=0.6)
    fig.tight_layout()
    return fig
