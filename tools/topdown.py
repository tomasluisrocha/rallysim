"""Top-down replay of a simulator CSV: path, heading, direction of travel, and the force acting on a rigid body.

Reads the telemetry only; it never touches the physics. It works on rigid-body files (quat, vel and
force columns) and on bicycle and single-track files (yaw, steer, speed and beta columns). A rigid body gets a
red force arrow; a bicycle gets a front wheel turned by the steer angle. Both get a green arrow along the
velocity of the CG, so the angle between it and the body is the sideslip beta.

  py tools/topdown.py out/st_circle.csv                           interactive window with a time slider
  py tools/topdown.py out/st_circle.csv --static                  whole path with arrows every N rows
  py tools/topdown.py out/st_circle.csv --save out/st_circle.png  write the static view to a file
"""
import argparse
import sys

import matplotlib
import numpy as np
import pandas as pd

BODY_LENGTH = 4.0   # m, drawn rectangle
BODY_WIDTH = 1.8    # m
WHEEL_LENGTH = 1.0  # m, drawn front wheel (bicycle files only)
TRAVEL_LENGTH = 5.0 # m, drawn direction-of-travel arrow (from the CG, so it reaches past the nose)
TRAVEL_COLOR = "tab:green"

RIGID_BODY = ["pos.x", "pos.y", "vel.x", "vel.y", "quat.w", "quat.x", "quat.y", "quat.z", "force.x", "force.y"]
BICYCLE = ["pos.x", "pos.y", "yaw", "steer", "speed", "beta"]


def load(path):
    df = pd.read_csv(path, comment="#")
    if all(c in df.columns for c in RIGID_BODY) or all(c in df.columns for c in BICYCLE):
        return df
    sys.exit(f"{path}: not a rigid-body file (needs {RIGID_BODY}) or a bicycle file (needs {BICYCLE}).")


def is_bicycle(df):
    return "yaw" in df.columns


def heading(df):
    """Direction of the body x axis in the world frame, projected on the ground (unit vectors)."""
    if is_bicycle(df):
        yaw = df["yaw"].to_numpy()
        return np.cos(yaw), np.sin(yaw)
    w, x, y, z = (df[f"quat.{c}"].to_numpy() for c in "wxyz")
    hx = 1 - 2 * (y * y + z * z)
    hy = 2 * (x * y + w * z)
    n = np.hypot(hx, hy)
    n[n == 0] = 1
    return hx / n, hy / n


def ground_velocity(df):
    """Velocity of the CG in the world frame, on the ground (m/s). Vehicle files store the speed and the
    sideslip beta, the angle from the heading to the velocity; rigid-body files store the velocity itself."""
    if is_bicycle(df):
        angle = df["yaw"].to_numpy() + df["beta"].to_numpy()
        speed = df["speed"].to_numpy()
        return speed * np.cos(angle), speed * np.sin(angle)
    return df["vel.x"].to_numpy(), df["vel.y"].to_numpy()


def travel(df):
    """Direction of travel of the CG as unit vectors; zero where the body is (almost) still."""
    vx, vy = ground_velocity(df)
    speed = np.hypot(vx, vy)
    moving = speed > 1e-3
    safe = np.where(moving, speed, 1.0)
    return np.where(moving, vx / safe, 0.0), np.where(moving, vy / safe, 0.0)


def lateral_acceleration(df):
    """Real lateral acceleration (m/s^2, positive to the left of the body): the change of the CG velocity
    over time, projected on the body's left axis. Unlike a_lat = speed * yaw_rate, which only holds in a
    steady turn, this stays right while the car slides or spins."""
    vx, vy = ground_velocity(df)
    t = df["time"].to_numpy()
    ax, ay = np.gradient(vx, t), np.gradient(vy, t)
    hx, hy = heading(df)
    return -ax * hy + ay * hx


def body_outline(px, py, hx, hy):
    """Corners of the body rectangle centred on (px, py), pointing along (hx, hy)."""
    lx, ly = hx, hy          # forward
    sx, sy = -hy, hx         # left
    a, b = BODY_LENGTH / 2, BODY_WIDTH / 2
    corners = [(a, b), (a, -b), (-a, -b), (-a, b), (a, b)]
    return ([px + c * lx + d * sx for c, d in corners],
            [py + c * ly + d * sy for c, d in corners])


def wheel_segment(px, py, yaw, steer):
    """End points of the front wheel: centred on the nose of the body, turned by `steer` from the heading."""
    hx, hy = np.cos(yaw), np.sin(yaw)
    cx, cy = px + hx * BODY_LENGTH / 2, py + hy * BODY_LENGTH / 2
    wx, wy = np.cos(yaw + steer) * WHEEL_LENGTH / 2, np.sin(yaw + steer) * WHEEL_LENGTH / 2
    return [cx - wx, cx + wx], [cy - wy, cy + wy]


def arrow_scale(df, step):
    """Metres of arrow per newton, so the longest sampled arrow is ~10% of the path size."""
    extent = max(np.ptp(df["pos.x"]), np.ptp(df["pos.y"]), 1.0)
    fmax = np.hypot(df["force.x"].to_numpy()[::step], df["force.y"].to_numpy()[::step]).max()
    return 0.1 * extent / fmax if fmax > 0 else 0.0


def setup_axes(plt, df, title):
    fig, ax = plt.subplots(figsize=(10, 5.5))
    ax.plot(df["pos.x"], df["pos.y"], color="0.75", lw=1, zorder=1)
    # room around the path for the body and the arrows, so they are not cut at the edges
    pad = BODY_LENGTH / 2 + TRAVEL_LENGTH
    ax.update_datalim([(df["pos.x"].min() - pad, df["pos.y"].min() - pad),
                       (df["pos.x"].max() + pad, df["pos.y"].max() + pad)])
    ax.autoscale_view()
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x forward (m)")
    ax.set_ylabel("y left (m)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    return fig, ax


def draw_static(plt, df, path, step):
    fig, ax = setup_axes(plt, df, f"{path}: every {step} rows")
    hx, hy = heading(df)
    idx = np.arange(0, len(df), step)
    px, py = df["pos.x"].to_numpy()[idx], df["pos.y"].to_numpy()[idx]
    ax.plot(px, py, "o", ms=3, color="tab:blue", zorder=3)
    ax.quiver(px, py, hx[idx], hy[idx], angles="xy", scale_units="xy", scale=1 / 1.2,
              width=0.003, color="0.25", zorder=2, label="heading (1.2 m)")
    tx, ty = travel(df)
    ax.quiver(px, py, tx[idx], ty[idx], angles="xy", scale_units="xy", scale=1 / 1.2,
              width=0.003, color=TRAVEL_COLOR, zorder=2, label="direction of travel (1.2 m)")

    if is_bicycle(df):
        yaw, steer = df["yaw"].to_numpy()[idx], df["steer"].to_numpy()[idx]
        for i in range(len(idx)):
            ax.plot(*wheel_segment(px[i], py[i], yaw[i], steer[i]), color="black", lw=3,
                    solid_capstyle="butt", zorder=5, label="front wheel" if i == 0 else None)
    else:
        k = arrow_scale(df, step)
        if k > 0:
            fx, fy = df["force.x"].to_numpy()[idx], df["force.y"].to_numpy()[idx]
            ax.quiver(px, py, fx * k, fy * k, angles="xy", scale_units="xy", scale=1,
                      width=0.004, color="tab:red", zorder=4,
                      label=f"force x-y ({1 / k:.0f} N per metre of arrow)")
        else:
            ax.text(0.5, 0.92, "no horizontal force in this run", transform=ax.transAxes, ha="center")
    ax.legend(loc="best")
    return fig


def draw_slider(plt, df, path, step):
    from matplotlib.widgets import Slider

    fig, ax = setup_axes(plt, df, path)
    fig.subplots_adjust(bottom=0.2)
    hx, hy = heading(df)
    px, py = df["pos.x"].to_numpy(), df["pos.y"].to_numpy()
    t = df["time"].to_numpy() if "time" in df.columns else np.arange(len(df))
    bicycle = is_bicycle(df)

    body, = ax.plot([], [], color="tab:blue", lw=2, zorder=3)
    trail, = ax.plot([], [], color="tab:blue", lw=1.5, zorder=2)
    tx, ty = travel(df)
    moving = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                         arrowprops=dict(arrowstyle="-|>", color=TRAVEL_COLOR, lw=2), zorder=4)
    ax.plot([], [], color=TRAVEL_COLOR, lw=2, label=f"direction of travel ({TRAVEL_LENGTH:.0f} m)")

    if bicycle:
        yaw, steer = df["yaw"].to_numpy(), df["steer"].to_numpy()
        speed, beta = df["speed"].to_numpy(), df["beta"].to_numpy()
        a_lat = lateral_acceleration(df)
        wheel, = ax.plot([], [], color="black", lw=4, solid_capstyle="butt", zorder=5, label="front wheel")
    else:
        k = arrow_scale(df, step)
        fx, fy = df["force.x"].to_numpy(), df["force.y"].to_numpy()
        arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                            arrowprops=dict(arrowstyle="-|>", color="tab:red", lw=2), zorder=4)
        ax.plot([], [], color="tab:red", lw=2,
                label=f"force x-y ({1 / k:.0f} N per metre)" if k > 0 else "force x-y (none)")
    ax.legend(loc="upper right")
    slider = Slider(fig.add_axes([0.12, 0.04, 0.76, 0.03]), "row", 0, len(df) - 1,
                    valinit=0, valstep=1)

    def update(val):
        i = int(slider.val)
        body.set_data(*body_outline(px[i], py[i], hx[i], hy[i]))
        trail.set_data(px[: i + 1], py[: i + 1])
        moving.xy = (px[i] + tx[i] * TRAVEL_LENGTH, py[i] + ty[i] * TRAVEL_LENGTH)
        moving.set_position((px[i], py[i]))
        if bicycle:
            wheel.set_data(*wheel_segment(px[i], py[i], yaw[i], steer[i]))
            ax.set_title(f"{path}   t = {t[i]:.3f} s   steer = {np.degrees(steer[i]):+.1f} deg   "
                         f"speed = {speed[i]:.1f} m/s   beta = {np.degrees(beta[i]):+.1f} deg   "
                         f"a_y = {a_lat[i]:+.2f} m/s^2", fontsize=9)
        else:
            arrow.xy = (px[i] + fx[i] * k, py[i] + fy[i] * k)
            arrow.set_position((px[i], py[i]))
            ax.set_title(f"{path}   t = {t[i]:.3f} s   force = ({fx[i]:.0f}, {fy[i]:.0f}) N")
        fig.canvas.draw_idle()

    slider.on_changed(update)
    update(0)
    fig._slider = slider  # keep a reference so it is not garbage collected
    return fig, update


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--every", type=int, default=500, metavar="N", help="rows between arrows (default 500)")
    ap.add_argument("--static", action="store_true", help="show the static view instead of the slider")
    ap.add_argument("--save", metavar="PNG", help="write the static view to a file and exit")
    args = ap.parse_args()
    if args.every < 1:
        sys.exit("--every must be at least 1")

    if args.save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    df = load(args.file)
    if args.save or args.static:
        fig = draw_static(plt, df, args.file, args.every)
        if args.save:
            fig.savefig(args.save, dpi=110, bbox_inches="tight")
            print(f"wrote {args.save}")
            return
    else:
        fig, _ = draw_slider(plt, df, args.file, args.every)
    plt.show()


if __name__ == "__main__":
    main()