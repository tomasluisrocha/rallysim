"""Where a four-wheel run's tyres sit on their force curve.

Every tyre, at every moment, is drawn as a point on its own curve: across, the normalised slip (the combined slip
the curve is read at); up, the grip it uses, |F| / (mu(Fz) Fz). For linear and peak_slide tyres the slip is
s = |(Cx sx, C_alpha sy)| / grip (1 = where the linear tyre reaches its grip), and every load and direction collapses
onto one curve. For tmeasy tyres it is sigma, each slip over its own peak slip (1 = the peak, in every direction),
and each direction has its own curve, drawn at the nominal load; points of a mixed slip, or at other loads, sit
near them. Braking hard, a point climbs the curve, passes the peak and slides down to the locked plateau;
cornering, the outer front sits just under its peak.

Usage:
  py tools/tyre_points.py out/runs/fw_brake.csv                      show it
  py tools/tyre_points.py out/runs/fw_brake.csv --save               write out/plots/tyres/fw_brake.png
  py tools/tyre_points.py out/runs/fw_brake.csv --save my.png --min-speed 2

The slips in the file are the steady (kinematic) ones; the force comes from the relaxed slips, which lag them
over the relaxation length. So while the slip is changing fast the points sit off the curve, and they land on it
once it settles. Below --min-speed (default 1 m/s) the steady slip means nothing and the points are left out.
"""
import argparse
import sys
from pathlib import Path

import matplotlib
import numpy as np

import simdata

COLOURS = {"FL": "tab:blue", "FR": "tab:orange", "RL": "tab:green", "RR": "tab:red"}


def wheel_points(data, tyre, wheel, moving):
    """Normalised slip and grip use of one wheel, NaN where the car is too slow or the wheel is unloaded."""
    col = lambda field: data[f"wheel.{wheel}.{field}"].to_numpy()
    fz = col("Fz")
    s = simdata.normalised_slip(tyre, col("slipRatio"), np.tan(col("slipAngle")), fz)
    g = simdata.grip(tyre, fz)
    with np.errstate(divide="ignore", invalid="ignore"):
        fx, fy = np.where(g > 0, col("Fx") / g, np.nan), np.where(g > 0, col("Fy") / g, np.nan)
    use = np.hypot(fx, fy)
    for values in (s, use, fx, fy):
        values[~moving] = np.nan
    return s, use, fx, fy


def plot(path, min_speed):
    data, meta = simdata.read_run(path)
    if not simdata.is_four_wheel(data):
        sys.exit(f"{path}: not a four-wheel run (no wheel.*.slipRatio columns)")
    tyres = {axle: simdata.tyre_from_meta(meta, axle) for axle in ("front", "rear")}
    if tyres["front"]["missing"]:
        print(f"note: {path} has no tyre keys on line 1 ({', '.join(tyres['front']['missing'])}); using the defaults")
    t = data["time"].to_numpy()
    moving = data["speed"].to_numpy() >= min_speed

    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    ax_curve, ax_circle, ax_s, ax_use = axes.ravel()

    # How far along the slip axis to draw: past the full-sliding point, but a locked wheel (s near 30) is pinned
    # to the edge rather than squashing the rest.
    edge = max(simdata.axis_edge(tyre) for tyre in tyres.values())
    thin = max(1, len(t) // 3000)
    for wheel in simdata.WHEELS:
        tyre = tyres["front" if wheel[0] == "F" else "rear"]
        s, use, fx, fy = wheel_points(data, tyre, wheel, moving)
        pinned = s > edge
        shown = np.where(pinned, edge, s)
        colour = COLOURS[wheel]
        ax_curve.scatter(shown[::thin], use[::thin], s=6, alpha=0.35, color=colour, label=wheel)
        if pinned.any():
            ax_curve.scatter(shown[pinned][::thin], use[pinned][::thin], s=30, marker=">", color=colour)
        ax_circle.scatter(fx[::thin], fy[::thin], s=6, alpha=0.35, color=colour, label=wheel)
        ax_s.plot(t, np.minimum(s, edge), color=colour, lw=1, label=wheel)
        ax_use.plot(t, use, color=colour, lw=1, label=wheel)

    # the curve(s) the points should sit on: one per axle, or one if both axles have the same tyre; a tmeasy tyre
    # has one per direction (the points of a mixed slip sit between them)
    same = all(tyres["front"].get(k) == tyres["rear"].get(k) for k in simdata.TYRE_DEFAULTS)
    styles = ["-", "--", ":", "-."]
    references = (simdata.reference_curves(tyres["front"], edge) if same else
                  simdata.reference_curves(tyres["front"], edge, "front") + simdata.reference_curves(tyres["rear"], edge, "rear"))
    for (name, sweep, curve), style in zip(references, styles * 2):
        ax_curve.plot(sweep, curve, style, color="black", lw=1.5, label=name + (" (both axles)" if same else ""))

    for value, text in simdata.marks(tyres["front"]):
        ax_curve.axvline(value, color="grey", ls=":", lw=1)
        ax_s.axhline(value, color="grey", ls=":", lw=1)
        ax_s.text(t[0], value, " " + text, va="bottom", fontsize=8, color="grey")

    name = simdata.label_of(path)
    ax_curve.set_title(f"{name}: every tyre on its curve (> = beyond the edge, e.g. locked)")
    ax_curve.set_xlabel(simdata.slip_axis_label(tyres["front"]))
    ax_curve.set_ylabel("grip used  |F| / mu(Fz) Fz")
    ax_curve.set_xlim(0, edge * 1.03)
    ax_curve.set_ylim(0, 1.15)
    ax_curve.legend(fontsize=8, loc="lower right")

    theta = np.linspace(0, 2 * np.pi, 361)
    ax_circle.plot(np.cos(theta), np.sin(theta), color="grey", lw=1)
    ax_circle.set_title("Force over grip: along (Fx) and across (Fy) the wheel")
    ax_circle.set_xlabel("Fx / grip  (+ driving, - braking)")
    ax_circle.set_ylabel("Fy / grip  (+ to the wheel's left)")
    ax_circle.set_aspect("equal", adjustable="datalim")
    ax_circle.legend(fontsize=8, loc="lower right")

    ax_s.set_title(f"Normalised slip over time (drawn up to {edge:.1f}; below {min_speed:g} m/s left out)")
    ax_s.set_xlabel("time [s]")
    ax_s.set_ylabel("s")
    ax_s.set_ylim(0, edge * 1.03)
    ax_s.legend(fontsize=8, loc="upper right")

    ax_use.set_title("Grip used over time")
    ax_use.set_xlabel("time [s]")
    ax_use.set_ylabel("|F| / grip")
    ax_use.set_ylim(0, 1.15)
    ax_use.legend(fontsize=8, loc="upper right")

    for ax in axes.ravel():
        ax.grid(True, alpha=0.4)
    fig.tight_layout()
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="a four-wheel telemetry CSV, e.g. out/runs/fw_brake.csv")
    ap.add_argument("--save", nargs="?", const="", metavar="PNG",
                    help="write the figure (default name: out/plots/tyres/<run>.png)")
    ap.add_argument("--min-speed", type=float, default=1.0, metavar="M/S", help="leave out slower moments (default 1)")
    args = ap.parse_args()

    error = simdata.check_against_rig()
    if error is not None and error > 1.0:
        print(f"warning: tools/simdata.py's curve differs from the C++ rig by up to {error:.1f} N; update it")

    if args.save is not None:
        matplotlib.use("Agg")
    fig = plot(args.file, args.min_speed)
    if args.save is None:
        import matplotlib.pyplot as plt
        plt.show()
        return
    out = Path(args.save) if args.save else simdata.TYRES / f"{Path(args.file).stem}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
