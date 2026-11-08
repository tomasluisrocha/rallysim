"""Plot the single-tyre rig (book section 7.2): force against slip for each tyre curve.

Usage:
    python tools/tyre_plot.py                      # reads out/rig/tyre_rig.csv, shows the figure
    python tools/tyre_plot.py --save out/plots/tyres/tyre_rig.png
    python tools/tyre_plot.py --curve peak_slide          # every panel on peak_slide
    python tools/tyre_plot.py --curve peak_slide brush --combined brush

Make the CSV first with:  build/Release/rallysim tyre_rig
Panels: Fx against slip ratio and Fy against slip angle at each load (colour = load, line style = curve),
and, for one curve, the combined sweeps: Fx and Fy against slip ratio at fixed slip angles, and Fy against Fx
with the grip circle mu Fz at the nominal load (the force must stay inside it). That one curve is --combined if
given, else the first --curve, else tmeasy (the car's).
"""
import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

STYLES = {"linear": "--", "brush": ":", "peak_slide": (0, (6, 2, 1, 2)), "tmeasy": "-"}


def main():
    parser = argparse.ArgumentParser(description="Plot the single-tyre rig CSV.")
    parser.add_argument("file", nargs="?", default="out/rig/tyre_rig.csv", help="rig CSV (default out/rig/tyre_rig.csv)")
    parser.add_argument("--curve", nargs="+", help="curves for the pure-slip panels (default: all in the file)")
    parser.add_argument("--combined", help="curve for the combined panels (default: the first --curve, else tmeasy, "
                                           "the car's)")
    parser.add_argument("--save", metavar="PNG", help="save the figure to this file instead of showing it")
    args = parser.parse_args()

    data = pd.read_csv(args.file, comment="#")
    curves = args.curve or list(dict.fromkeys(data["curve"]))
    # The combined panels show one curve: the one asked for, else the first curve picked for the pure-slip panels,
    # else the car's. (They used to default to tmeasy even with --curve peak_slide, which hid which curve they were.)
    if args.combined is None:
        args.combined = args.curve[0] if args.curve else "tmeasy"
    unknown = [c for c in dict.fromkeys(curves + [args.combined]) if c not in set(data["curve"])]
    if unknown:
        print("Unknown curve(s): " + ", ".join(unknown))
        print("In the file: " + ", ".join(dict.fromkeys(data["curve"])))
        sys.exit(1)

    loads = sorted(data.loc[data["sweep"] == "x", "fz"].unique())
    load_colour = {fz: plt.cm.viridis(i / max(len(loads) - 1, 1)) for i, fz in enumerate(loads)}

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    ax_x, ax_y, ax_c, ax_circle = axes.ravel()

    for curve in curves:
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
    ax_x.legend(fontsize=7, ncol=len(curves))

    combined = data[(data["curve"] == args.combined) & (data["sweep"] == "combined")]
    angles = sorted(combined["slip_angle_deg"].unique())
    angle_colour = {a: plt.cm.plasma(i / max(len(angles) - 1, 1) * 0.85) for i, a in enumerate(angles)}
    for angle in angles:
        rows = combined[combined["slip_angle_deg"] == angle]
        ax_c.plot(rows["slip_ratio"], rows["fx"], "-", color=angle_colour[angle], label=f"Fx, {angle:g} deg")
        ax_c.plot(rows["slip_ratio"], rows["fy"], "--", color=angle_colour[angle], label=f"Fy, {angle:g} deg")
        ax_circle.plot(rows["fx"], rows["fy"], "-", color=angle_colour[angle], label=f"{angle:g} deg")
    ax_c.set_title(f"Combined slip ({args.combined}): the side force falls as the wheel slips")
    ax_c.set_xlabel("slip ratio")
    ax_c.set_ylabel("force [N]")
    ax_c.legend(fontsize=7, ncol=2)

    # Grip at the combined sweep's load: the linear tyre's pure-slip maximum there is exactly mu Fz.
    fz_c = combined["fz"].iloc[0]
    reference = data[(data["sweep"] == "x") & (data["fz"] == fz_c)]
    linear = reference[reference["curve"] == "linear"]
    grip = (linear if len(linear) else reference)["fx"].abs().max()
    theta = np.linspace(0, 2 * np.pi, 361)
    ax_circle.plot(grip * np.cos(theta), grip * np.sin(theta), color="grey", lw=1, label=f"mu Fz = {grip:.0f} N")
    ax_circle.set_title(f"Combined slip ({args.combined}): Fy against Fx, Fz {fz_c:.0f} N")
    ax_circle.set_xlabel("Fx [N]")
    ax_circle.set_ylabel("Fy [N]")
    ax_circle.set_aspect("equal", adjustable="datalim")
    ax_circle.legend(fontsize=7)

    for ax in axes.ravel():
        ax.grid(True, alpha=0.4)
        ax.axhline(0, color="black", lw=0.6)
        ax.axvline(0, color="black", lw=0.6)
    fig.tight_layout()

    if args.save:
        os.makedirs(os.path.dirname(args.save) or ".", exist_ok=True)
        fig.savefig(args.save, dpi=110)
        print(f"saved {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
