"""Top-down viewer for simulator runs: the car to scale, every tyre's force and grip, and the run around it.

Reads the telemetry only; it never touches the physics. It recognises each kind of file by its columns:

  four-wheel car    four wheels at their real positions, each with its tyre force and how much grip it uses
  single-track      two wheels on the centreline (one per axle), with the axle forces and grip
  bicycle           two wheels on the centreline, no forces (the kinematic model has none)
  rigid body        a box with the total applied force

The car's geometry comes from line 1 of the file (lf, lr, tracks, tyre grip); older files get the default car.

Usage:
  py tools/topdown.py out/runs/fw_handbrake.csv                dashboard: play, pause, scrub
  py tools/topdown.py out/runs/fw_handbrake.csv --at 3.0 --save out/scratch/frame.png one dashboard frame
  py tools/topdown.py out/runs/fw_handbrake.csv --static --save out/plots/views/fw_handbrake.png the path with snapshots
  py tools/topdown.py out/runs/fw_handbrake.csv --gif out/gif/fw_handbrake.gif --start 2 --end 5 the dashboard as a GIF

Dashboard keys: space play/pause, left/right 0.1 s (shift: 1 s), up/down playback speed x2 / /2, home/end.
"""
import argparse
import math
import sys
from dataclasses import dataclass, field

import matplotlib
import numpy as np
import pandas as pd

# Drawing sizes (m) for parts the telemetry does not describe.
OVERHANG = 0.8           # body beyond each axle
BODY_WIDTH_MARGIN = 0.25  # body wider than the track
TYRE_WIDTH = 0.22
TRAVEL_ARROW = 3.5       # m: length of the direction-of-travel arrow (the speed is in the title)
STILL_SPEED = 0.3        # m/s: below this the sideslip angle means nothing and is not shown
FORCE_ARROW_MAX = 2.5    # m: the largest tyre force in the run is drawn this long
LOCK_SPEED = 0.5         # m/s: a wheel that is stopped while the car moves faster than this is locked
SPIN_SLIP = 0.15         # slip ratio beyond which a wheel counts as spinning (or sliding when negative)

# The default car, for files written before the geometry was put on line 1.
DEFAULT_META = {"lf": 1.3, "lr": 1.2, "track_front": 1.5, "track_rear": 1.5, "max_steer": 0.6,
                "wheel_radius": 0.315, "mu_front": 1.0, "k_front": 0.1, "fz0_front": 3000.0,
                "mu_rear": 1.0, "k_rear": 0.1, "fz0_rear": 3000.0}


# ----------------------------------------------------------------------------------------------------------
# Reading a run into one description, whatever the model
# ----------------------------------------------------------------------------------------------------------

@dataclass
class Wheel:
    label: str
    x: float                      # contact patch from the CG, body frame (m)
    y: float
    steer: np.ndarray             # rad, per row
    fx: np.ndarray = None         # tyre force along the wheel (N), per row; None if the model has no forces
    fy: np.ndarray = None         # tyre force across the wheel, to its left (N)
    fz: np.ndarray = None         # vertical load (N)
    grip: np.ndarray = None       # largest force the tyre can take (N)
    omega: np.ndarray = None      # spin (rad/s)
    slip_ratio: np.ndarray = None


@dataclass
class Channel:
    name: str
    values: np.ndarray
    low: float                    # the range the bar is drawn over
    high: float
    fmt: str = "{:+.2f}"


@dataclass
class Run:
    path: str
    kind: str
    t: np.ndarray
    x: np.ndarray                 # CG, world (m)
    y: np.ndarray
    yaw: np.ndarray               # heading (rad)
    vx: np.ndarray                # CG velocity, world (m/s)
    vy: np.ndarray
    speed: np.ndarray
    beta: np.ndarray              # sideslip (rad)
    yaw_rate: np.ndarray          # rad/s
    a_lat: np.ndarray             # lateral acceleration from the motion (m/s^2)
    body_length: float
    body_width: float
    body_front: float             # body front from the CG (m)
    tyre_length: float
    wheels: list = field(default_factory=list)
    inputs: list = field(default_factory=list)
    body_force: tuple = None      # (fx, fy) world, for rigid bodies


def read_meta(path):
    """key=value pairs on line 1, between dt and the units."""
    with open(path, encoding="utf-8") as f:
        first = f.readline()
    meta = {}
    if first.startswith("#"):
        for token in first[1:].split("units:")[0].split():
            key, sep, value = token.partition("=")
            if sep:
                try:
                    meta[key] = float(value)
                except ValueError:
                    pass
    return meta


def lateral_acceleration(t, vx, vy, yaw):
    """Change of the CG velocity over time, along the body's left axis (positive left)."""
    if len(t) < 2:
        return np.zeros_like(t)
    ax, ay = np.gradient(vx, t), np.gradient(vy, t)
    return -ax * np.sin(yaw) + ay * np.cos(yaw)


def sideslip(beta, speed):
    """The sideslip angle, blank (NaN) where the car is (almost) still."""
    return np.where(speed > STILL_SPEED, beta, np.nan)


def symmetric(values, floor):
    m = float(np.nanmax(np.abs(values))) if len(values) else 0.0
    return max(m, floor)


def load(path):
    df = pd.read_csv(path, comment="#")
    meta = dict(DEFAULT_META)
    meta.update(read_meta(path))
    cols = set(df.columns)
    t = df["time"].to_numpy() if "time" in cols else np.arange(len(df), dtype=float)

    if "quat.w" in cols:
        return load_rigid_body(path, df, t)
    if "yaw" not in cols or "speed" not in cols:
        sys.exit(f"{path}: not a file this viewer knows (no quat.* or yaw/speed columns)")

    yaw = df["yaw"].to_numpy()
    speed = df["speed"].to_numpy()
    beta = df["beta"].to_numpy() if "beta" in cols else np.zeros_like(yaw)
    vx, vy = speed * np.cos(yaw + beta), speed * np.sin(yaw + beta)
    lf, lr = meta["lf"], meta["lr"]
    tf, tr = meta["track_front"], meta["track_rear"]
    run = Run(path=path, kind="", t=t, x=df["pos.x"].to_numpy(), y=df["pos.y"].to_numpy(), yaw=yaw,
              vx=vx, vy=vy, speed=speed, beta=sideslip(beta, speed), yaw_rate=df["yaw_rate"].to_numpy(),
              a_lat=lateral_acceleration(t, vx, vy, yaw),
              body_length=lf + lr + 2 * OVERHANG, body_width=max(tf, tr) + BODY_WIDTH_MARGIN,
              body_front=lf + OVERHANG, tyre_length=2 * meta["wheel_radius"])
    steer = df["steer"].to_numpy()
    zero = np.zeros_like(steer)
    steer_channel = Channel("steer (deg)", np.degrees(steer), -math.degrees(meta["max_steer"]),
                            math.degrees(meta["max_steer"]), "{:+.1f}")

    if "wheel.FL.Fx" in cols:
        run.kind = "four-wheel car"
        positions = {"FL": (lf, tf / 2), "FR": (lf, -tf / 2), "RL": (-lr, tr / 2), "RR": (-lr, -tr / 2)}
        for label, (x, y) in positions.items():
            end = "front" if label[0] == "F" else "rear"
            fz = df[f"wheel.{label}.Fz"].to_numpy()
            mu, k, fz0 = meta[f"mu_{end}"], meta[f"k_{end}"], meta[f"fz0_{end}"]
            grip = mu * np.clip(1 - k * (fz - fz0) / fz0, 0, None) * np.clip(fz, 0, None)
            run.wheels.append(Wheel(label, x, y, steer if end == "front" else zero,
                                    df[f"wheel.{label}.Fx"].to_numpy(), df[f"wheel.{label}.Fy"].to_numpy(),
                                    fz, grip, df[f"wheel.{label}.omega"].to_numpy(),
                                    df[f"wheel.{label}.slipRatio"].to_numpy()))
        run.inputs = [steer_channel] + [Channel(p, df[p].to_numpy(), 0, 1, "{:.2f}")
                                        for p in ("throttle", "brake", "handbrake")]
    elif "axle.front.Fy" in cols:
        run.kind = "single-track"
        front_fz = (df["wheel.FL.Fz"] + df["wheel.FR.Fz"]).to_numpy() if "wheel.FL.Fz" in cols else None
        rear_fz = (df["wheel.RL.Fz"] + df["wheel.RR.Fz"]).to_numpy() if "wheel.RL.Fz" in cols else None
        rear_fx = df["axle.rear.Fx"].to_numpy() if "axle.rear.Fx" in cols else zero
        run.wheels = [
            Wheel("front", lf, 0.0, steer, zero, df["axle.front.Fy"].to_numpy(), front_fz,
                  df["axle.front.grip"].to_numpy() if "axle.front.grip" in cols else None),
            Wheel("rear", -lr, 0.0, zero, rear_fx, df["axle.rear.Fy"].to_numpy(), rear_fz,
                  df["axle.rear.grip"].to_numpy() if "axle.rear.grip" in cols else None)]
        drive = df["drive_force"].to_numpy()
        run.inputs = [steer_channel, Channel("drive force (N)", drive, -symmetric(drive, 1.0), symmetric(drive, 1.0), "{:+.0f}")]
    else:
        run.kind = "kinematic bicycle"
        run.wheels = [Wheel("front", lf, 0.0, steer), Wheel("rear", -lr, 0.0, zero)]
        run.inputs = [steer_channel]
        if "accel" in cols:
            accel = df["accel"].to_numpy()
            run.inputs.append(Channel("accel (m/s^2)", accel, -symmetric(accel, 0.1), symmetric(accel, 0.1)))
    return run


def load_rigid_body(path, df, t):
    w, x, y, z = (df[f"quat.{c}"].to_numpy() for c in "wxyz")
    yaw = np.arctan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z))
    vx, vy = df["vel.x"].to_numpy(), df["vel.y"].to_numpy()
    forward = np.cos(yaw) * vx + np.sin(yaw) * vy
    left = -np.sin(yaw) * vx + np.cos(yaw) * vy
    speed = np.hypot(vx, vy)
    fx = df["force.x"].to_numpy() if "force.x" in df.columns else np.zeros_like(t)
    fy = df["force.y"].to_numpy() if "force.y" in df.columns else np.zeros_like(t)
    run = Run(path=path, kind="rigid body", t=t, x=df["pos.x"].to_numpy(), y=df["pos.y"].to_numpy(), yaw=yaw,
              vx=vx, vy=vy, speed=speed, beta=sideslip(np.arctan2(left, forward), speed),
              yaw_rate=df["omega.z"].to_numpy(), a_lat=lateral_acceleration(t, vx, vy, yaw),
              body_length=4.0, body_width=1.8, body_front=2.0, tyre_length=0.0, body_force=(fx, fy))
    run.inputs = [Channel("force x (N)", fx, -symmetric(fx, 1.0), symmetric(fx, 1.0), "{:+.0f}"),
                  Channel("force y (N)", fy, -symmetric(fy, 1.0), symmetric(fy, 1.0), "{:+.0f}")]
    if "torque.z" in df.columns:
        tz = df["torque.z"].to_numpy()
        run.inputs.append(Channel("torque z (N m)", tz, -symmetric(tz, 1.0), symmetric(tz, 1.0), "{:+.0f}"))
    return run


def force_scale(run):
    """Metres of arrow per newton, so the largest force in the run is FORCE_ARROW_MAX long."""
    largest = 0.0
    for wh in run.wheels:
        if wh.fx is not None:
            largest = max(largest, float(np.nanmax(np.hypot(wh.fx, wh.fy))))
    if run.body_force is not None:
        largest = max(largest, float(np.nanmax(np.hypot(*run.body_force))))
    return FORCE_ARROW_MAX / largest if largest > 0 else 0.0


def has_grip(run):
    return any(wh.grip is not None and wh.fx is not None for wh in run.wheels)


def grip_use(wheel, i):
    """Share of the tyre's grip in use, 0..1, or None if the file does not say."""
    if wheel.grip is None or wheel.fx is None:
        return None
    g = wheel.grip[i]
    return min(float(np.hypot(wheel.fx[i], wheel.fy[i]) / g), 1.0) if g > 0 else 1.0


def wheel_state(run, wheel, i):
    """'LOCK', 'SPIN', 'SLIDE' or '' for a wheel at row i."""
    if wheel.omega is not None and wheel.omega[i] == 0 and abs(run.speed[i]) > LOCK_SPEED:
        return "LOCK"
    # slip means nothing when nothing rolls: at a standstill (omega R - vx) / max(|vx|, |omega R|) is 0/0 and reads +-1
    rolling = max(abs(run.speed[i]), abs(wheel.omega[i]) * run.tyre_length / 2 if wheel.omega is not None else 0.0)
    if wheel.slip_ratio is not None and rolling > LOCK_SPEED:
        if wheel.slip_ratio[i] > SPIN_SLIP:
            return "SPIN"
        if wheel.slip_ratio[i] < -SPIN_SLIP:
            return "SLIDE"
    return ""


def row_at(run, time):
    return int(np.clip(np.searchsorted(run.t, time), 0, len(run.t) - 1))


# ----------------------------------------------------------------------------------------------------------
# Drawing the car
# ----------------------------------------------------------------------------------------------------------

def rotate(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c * x - s * y, s * x + c * y


def rectangle(cx, cy, length, width, angle):
    """Corners of a rectangle centred on (cx, cy), its length along `angle`."""
    a, b = length / 2, width / 2
    pts = [rotate(px, py, angle) for px, py in ((a, b), (a, -b), (-a, -b), (-a, b))]
    return np.array([(cx + px, cy + py) for px, py in pts])


class CarArtist:
    """The car at one row: body, wheels coloured by grip in use, tyre force arrows, velocity arrow."""

    def __init__(self, ax, run, scale, cmap, alpha=1.0, labels=False):
        from matplotlib import patheffects
        from matplotlib.patches import FancyArrowPatch, Polygon
        self.ax, self.run, self.scale, self.cmap = ax, run, scale, cmap
        outline = [patheffects.Stroke(linewidth=3.8, foreground="0.15"), patheffects.Normal()]
        self.body = Polygon(np.zeros((4, 2)), closed=True, facecolor="#dfe7f2", edgecolor="#2b4a74",
                            lw=1.5, alpha=alpha, zorder=4)
        ax.add_patch(self.body)
        self.nose = Polygon(np.zeros((3, 2)), closed=True, facecolor="#2b4a74", alpha=alpha, zorder=5)
        ax.add_patch(self.nose)
        self.tyres, self.forces, self.labels = [], [], []
        for wheel in run.wheels:
            tyre = Polygon(np.zeros((4, 2)), closed=True, facecolor="0.3", edgecolor="black", lw=1.0,
                           alpha=alpha, zorder=6)
            ax.add_patch(tyre)
            self.tyres.append(tyre)
            arrow = FancyArrowPatch((0, 0), (0, 0), arrowstyle="-|>", mutation_scale=12, lw=2.2,
                                    color="black", alpha=alpha, zorder=8)
            arrow.set_path_effects(outline)  # a pale arrow (half the grip in use) stays visible on the body
            ax.add_patch(arrow)
            self.forces.append(arrow)
            self.labels.append(ax.text(0, 0, "", fontsize=7, fontweight="bold", color="black", ha="center",
                                       va="center", zorder=9, visible=labels))
        self.velocity = FancyArrowPatch((0, 0), (0, 0), arrowstyle="-|>", mutation_scale=14, lw=2.0,
                                        color="tab:green", alpha=alpha, zorder=7)
        ax.add_patch(self.velocity)
        self.body_force = None
        if run.body_force is not None:
            self.body_force = FancyArrowPatch((0, 0), (0, 0), arrowstyle="-|>", mutation_scale=14, lw=2.4,
                                              color="tab:red", alpha=alpha, zorder=8)
            ax.add_patch(self.body_force)
        self.show_labels = labels

    def artists(self):
        a = [self.body, self.nose, self.velocity] + self.tyres + self.forces + self.labels
        return a + ([self.body_force] if self.body_force is not None else [])

    @staticmethod
    def _arrow(arrow, x0, y0, dx, dy):
        if math.hypot(dx, dy) < 1e-3:
            arrow.set_visible(False)
        else:
            arrow.set_visible(True)
            arrow.set_positions((x0, y0), (x0 + dx, y0 + dy))

    def update(self, i):
        r = self.run
        cx, cy, yaw = r.x[i], r.y[i], r.yaw[i]
        centre = r.body_front - r.body_length / 2  # body centre from the CG, along the body
        bx, by = rotate(centre, 0.0, yaw)
        self.body.set_xy(rectangle(cx + bx, cy + by, r.body_length, r.body_width, yaw))
        tip = rotate(r.body_front, 0.0, yaw)
        side = rotate(r.body_front - 0.6, 0.35 * r.body_width, yaw)
        other = rotate(r.body_front - 0.6, -0.35 * r.body_width, yaw)
        self.nose.set_xy(np.array([(cx + tip[0], cy + tip[1]), (cx + side[0], cy + side[1]),
                                   (cx + other[0], cy + other[1])]))

        for wheel, tyre, arrow, label in zip(r.wheels, self.tyres, self.forces, self.labels):
            px, py = rotate(wheel.x, wheel.y, yaw)
            px, py = cx + px, cy + py
            angle = yaw + wheel.steer[i]
            tyre.set_xy(rectangle(px, py, max(r.tyre_length, 0.5), TYRE_WIDTH, angle))
            use = grip_use(wheel, i)
            tyre.set_facecolor(self.cmap(use) if use is not None else "0.3")
            state = wheel_state(r, wheel, i)
            tyre.set_hatch("////" if state == "LOCK" else "")
            tyre.set_edgecolor({"LOCK": "black", "SPIN": "magenta", "SLIDE": "darkorange"}.get(state, "black"))
            tyre.set_linewidth(2.2 if state else 1.0)
            if wheel.fx is not None and self.scale > 0:
                fx, fy = rotate(wheel.fx[i], wheel.fy[i], angle)
                self._arrow(arrow, px, py, fx * self.scale, fy * self.scale)
                arrow.set_color(self.cmap(use) if use is not None else "black")
            else:
                arrow.set_visible(False)
            if self.show_labels:
                lx, ly = rotate(wheel.x, wheel.y + math.copysign(0.75, wheel.y if wheel.y else 1), yaw)
                label.set_position((cx + lx, cy + ly))
                label.set_text(state)

        if r.speed[i] > STILL_SPEED:
            self._arrow(self.velocity, cx, cy, r.vx[i] / r.speed[i] * TRAVEL_ARROW, r.vy[i] / r.speed[i] * TRAVEL_ARROW)
        else:
            self.velocity.set_visible(False)
        if self.body_force is not None:
            fx, fy = r.body_force
            self._arrow(self.body_force, cx, cy, fx[i] * self.scale, fy[i] * self.scale)


def grip_cmap():
    import matplotlib.pyplot as plt
    return plt.get_cmap("RdYlGn_r")


def legend_text(run, scale):
    parts = ["green arrow: direction of travel (its angle to the body is the sideslip)"]
    if scale > 0:
        parts.append(f"force arrows: 1 m = {1 / scale:,.0f} N")
    if any(w.omega is not None for w in run.wheels):
        parts.append("hatched: locked, magenta: spinning, orange: sliding")
    return "\n".join(parts)


# ----------------------------------------------------------------------------------------------------------
# The dashboard
# ----------------------------------------------------------------------------------------------------------

class Dashboard:
    def __init__(self, plt, run, window, follow):
        from matplotlib.gridspec import GridSpec
        self.plt, self.run, self.window, self.follow = plt, run, window, follow
        self.cmap = grip_cmap()
        self.scale = force_scale(run)
        self.i = 0
        self.playing = False
        self.speed = 1.0

        self.fig = plt.figure(figsize=(15, 8.6))
        gs = GridSpec(3, 3, figure=self.fig, width_ratios=[1, 1, 0.9], height_ratios=[1, 1, 0.75],
                      left=0.05, right=0.96, top=0.92, bottom=0.13, wspace=0.45, hspace=0.38)
        self.ax_main = self.fig.add_subplot(gs[0:2, 0:2])
        self.ax_map = self.fig.add_subplot(gs[0, 2])
        self.ax_wheels = self.fig.add_subplot(gs[1, 2])
        self.ax_time = self.fig.add_subplot(gs[2, 0:2])
        self.ax_inputs = self.fig.add_subplot(gs[2, 2])

        self._setup_main()
        self._setup_map()
        self._setup_wheels()
        self._setup_time()
        self._setup_inputs()
        self.title = self.fig.suptitle("", fontsize=11, x=0.05, ha="left")

    # --- panels -------------------------------------------------------------------------------------------

    def _setup_main(self):
        r, ax = self.run, self.ax_main
        ax.plot(r.x, r.y, color="0.82", lw=1, zorder=1)
        self.trail, = ax.plot([], [], color="tab:blue", lw=1.8, zorder=2)
        self.car = CarArtist(ax, r, self.scale, self.cmap, labels=True)
        # equal scale on both axes, filling the panel: the y range follows the panel's shape
        box = ax.get_position()
        width, height = self.fig.get_size_inches()
        self.shape = (box.height * height) / (box.width * width)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.grid(True, alpha=0.3)
        ax.text(0.01, 0.01, legend_text(r, self.scale), transform=ax.transAxes, fontsize=8, va="bottom",
                bbox=dict(facecolor="white", alpha=0.8, edgecolor="0.8"), zorder=20)
        if has_grip(r):
            sm = matplotlib.cm.ScalarMappable(cmap=self.cmap, norm=matplotlib.colors.Normalize(0, 100))
            cax = ax.inset_axes([0.70, 0.04, 0.27, 0.025])
            bar = self.fig.colorbar(sm, cax=cax, orientation="horizontal")
            bar.set_label("tyre grip in use (%)", fontsize=8)
            bar.ax.tick_params(labelsize=7)
        if not self.follow:
            pad = r.body_length + 2
            cx, cy = (r.x.min() + r.x.max()) / 2, (r.y.min() + r.y.max()) / 2
            half = max(r.x.max() - r.x.min(), (r.y.max() - r.y.min()) / self.shape) / 2 + pad
            ax.set_xlim(cx - half, cx + half)
            ax.set_ylim(cy - half * self.shape, cy + half * self.shape)

    def _setup_map(self):
        r, ax = self.run, self.ax_map
        ax.plot(r.x, r.y, color="tab:blue", lw=1)
        ax.plot(r.x[0], r.y[0], "o", color="tab:green", ms=5)
        self.map_dot, = ax.plot([], [], "o", color="tab:red", ms=7, zorder=5)
        from matplotlib.patches import Rectangle
        self.map_window = Rectangle((0, 0), self.window, self.window, fill=False, ec="tab:red", lw=1,
                                    visible=self.follow)
        ax.add_patch(self.map_window)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_title("whole run", fontsize=9)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)

    def _setup_wheels(self):
        r, ax = self.run, self.ax_wheels
        self.bars, self.bar_text = [], []
        loaded = [w for w in r.wheels if w.fz is not None]
        if not loaded:
            ax.set_axis_off()
            ax.text(0.5, 0.5, f"{r.kind}:\nno tyre loads in this file", ha="center", va="center",
                    transform=ax.transAxes, fontsize=9, color="0.4")
            return
        top = max(float(np.nanmax(w.fz)) for w in loaded) / 1000 * 1.25
        labels = [w.label for w in loaded]
        self.bars = list(ax.bar(labels, [0] * len(labels), color="0.6", edgecolor="black"))
        self.bar_text = [ax.text(k, 0, "", ha="center", va="bottom", fontsize=8) for k in range(len(labels))]
        ax.set_ylim(0, top)
        ax.set_ylabel("load Fz (kN)", fontsize=8)
        ax.set_title("load per wheel, colour = grip in use", fontsize=9)
        ax.tick_params(labelsize=8)
        ax.grid(True, axis="y", alpha=0.3)
        self.loaded = loaded

    def _setup_time(self):
        r, ax = self.run, self.ax_time
        ax.plot(r.t, np.degrees(r.beta), color="tab:purple", lw=1.2, label="sideslip β (deg)")
        ax.plot(r.t, np.degrees(r.yaw_rate), color="black", lw=1.2, label="yaw rate (deg/s)")
        ax.set_xlabel("time (s)", fontsize=8)
        ax.set_ylabel("deg, deg/s", fontsize=8)
        ax.tick_params(labelsize=8)
        ax.grid(True, alpha=0.3)
        twin = ax.twinx()
        twin.plot(r.t, r.speed, color="tab:blue", lw=1.2, label="speed (m/s)")
        twin.set_ylabel("m/s", fontsize=8, color="tab:blue")
        twin.tick_params(labelsize=8, colors="tab:blue")
        lines = ax.get_lines() + twin.get_lines()
        ax.legend(lines, [l.get_label() for l in lines], fontsize=7, loc="upper left", ncol=3)
        self.cursor = ax.axvline(r.t[0], color="tab:red", lw=1)

    def _setup_inputs(self):
        r, ax = self.run, self.ax_inputs
        names = [c.name for c in r.inputs]
        self.input_bars = list(ax.barh(names, [0] * len(names), left=0, color="tab:blue", height=0.6))
        self.input_text = [ax.text(1.02, k, "", va="center", fontsize=8) for k in range(len(names))]
        ax.set_xlim(-1, 1)
        ax.axvline(0, color="black", lw=0.8)
        ax.invert_yaxis()
        ax.set_title("driver inputs (bar: share of its range)", fontsize=9)
        ax.tick_params(labelsize=8)
        ax.set_xticks([-1, 0, 1])
        ax.grid(True, axis="x", alpha=0.3)

    # --- one frame ----------------------------------------------------------------------------------------

    def draw(self, i):
        r = self.run
        self.i = i
        self.car.update(i)
        start = row_at(r, r.t[i] - 3.0)
        self.trail.set_data(r.x[start:i + 1], r.y[start:i + 1])
        if self.follow:
            h, v = self.window / 2, self.window / 2 * self.shape
            self.ax_main.set_xlim(r.x[i] - h, r.x[i] + h)
            self.ax_main.set_ylim(r.y[i] - v, r.y[i] + v)
            self.map_window.set_xy((r.x[i] - h, r.y[i] - v))
            self.map_window.set_height(2 * v)
        self.map_dot.set_data([r.x[i]], [r.y[i]])
        self.cursor.set_xdata([r.t[i], r.t[i]])

        for k, w in enumerate(getattr(self, "loaded", [])):
            fz = w.fz[i] / 1000
            self.bars[k].set_height(fz)
            use = grip_use(w, i)
            self.bars[k].set_facecolor(self.cmap(use) if use is not None else "0.6")
            state = wheel_state(r, w, i)
            text = f"{fz:.2f} kN" + (f"\n{100 * use:.0f}%" if use is not None else "") + (f"\n{state}" if state else "")
            self.bar_text[k].set_position((k, fz))
            self.bar_text[k].set_text(text)

        for bar, text, c in zip(self.input_bars, self.input_text, r.inputs):
            v = c.values[i]
            if c.low < 0:
                share = v / max(abs(c.low), abs(c.high))
            else:
                share = (v - c.low) / (c.high - c.low) if c.high > c.low else 0
            bar.set_width(float(np.clip(share, -1, 1)))
            text.set_text(c.fmt.format(v))

        beta_text = "—" if np.isnan(r.beta[i]) else f"{math.degrees(r.beta[i]):+.1f}°"
        self.title.set_text(f"{r.path}   [{r.kind}]   t = {r.t[i]:.2f} s   speed {r.speed[i]:.1f} m/s   "
                            f"β {beta_text}   yaw rate {math.degrees(r.yaw_rate[i]):+.1f}°/s   "
                            f"a_y {r.a_lat[i]:+.2f} m/s²" + ("   ▶" if self.playing else ""))

    # --- interactive controls -----------------------------------------------------------------------------

    def interactive(self):
        from matplotlib.widgets import Button, Slider
        r = self.run
        self.slider = Slider(self.fig.add_axes([0.12, 0.035, 0.55, 0.025]), "time (s)", r.t[0], r.t[-1],
                             valinit=r.t[0])
        self.slider.on_changed(lambda v: (self.draw(row_at(r, v)), self.fig.canvas.draw_idle()))
        self.play_button = Button(self.fig.add_axes([0.70, 0.025, 0.06, 0.045]), "play")
        self.play_button.on_clicked(lambda _: self.toggle())
        slower = Button(self.fig.add_axes([0.77, 0.025, 0.05, 0.045]), "slower")
        slower.on_clicked(lambda _: self.set_speed(self.speed / 2))
        faster = Button(self.fig.add_axes([0.83, 0.025, 0.05, 0.045]), "faster")
        faster.on_clicked(lambda _: self.set_speed(self.speed * 2))
        self.speed_text = self.fig.text(0.89, 0.04, "", fontsize=9)
        self._keep = (slower, faster)
        self.fig.canvas.mpl_connect("key_press_event", self.key)
        self.timer = self.fig.canvas.new_timer(interval=40)
        self.timer.add_callback(self.tick)
        self.timer.start()
        self.set_speed(1.0)
        self.draw(0)

    def set_speed(self, s):
        self.speed = float(np.clip(s, 1 / 64, 16))
        self.speed_text.set_text(f"playback ×{self.speed:g}")
        self.fig.canvas.draw_idle()

    def toggle(self):
        if not self.playing and self.i >= len(self.run.t) - 1:
            self.slider.set_val(self.run.t[0])
        self.playing = not self.playing
        self.play_button.label.set_text("pause" if self.playing else "play")
        self.draw(self.i)
        self.fig.canvas.draw_idle()

    def tick(self):
        if not self.playing:
            return
        t = self.run.t[self.i] + 0.04 * self.speed
        if t >= self.run.t[-1]:
            t = self.run.t[-1]
            self.playing = False
            self.play_button.label.set_text("play")
        self.slider.set_val(t)

    def key(self, event):
        r, t = self.run, self.run.t[self.i]
        steps = {"right": 0.1, "left": -0.1, "shift+right": 1.0, "shift+left": -1.0}
        if event.key == " ":
            self.toggle()
        elif event.key in steps:
            self.slider.set_val(float(np.clip(t + steps[event.key], r.t[0], r.t[-1])))
        elif event.key == "up":
            self.set_speed(self.speed * 2)
        elif event.key == "down":
            self.set_speed(self.speed / 2)
        elif event.key == "home":
            self.slider.set_val(r.t[0])
        elif event.key == "end":
            self.slider.set_val(r.t[-1])


# ----------------------------------------------------------------------------------------------------------
# The static view: the path coloured by speed, with the car drawn at intervals
# ----------------------------------------------------------------------------------------------------------

def draw_static(plt, run, every):
    from matplotlib.collections import LineCollection
    fig, ax = plt.subplots(figsize=(12, 7.5))
    points = np.column_stack([run.x, run.y]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    path = LineCollection(segments, cmap="viridis", lw=2.5, zorder=2)
    path.set_array(run.speed[:-1])
    ax.add_collection(path)
    fig.colorbar(path, ax=ax, orientation="horizontal", fraction=0.04, pad=0.09, aspect=50).set_label("path colour: speed (m/s)")

    scale = force_scale(run)
    cmap = grip_cmap()
    duration = run.t[-1] - run.t[0]
    every = every if every else max(round(duration / 10, 1), 0.1)
    times = np.arange(run.t[0], run.t[-1] + 1e-9, every)
    for k, time in enumerate(times):
        car = CarArtist(ax, run, scale, cmap, alpha=0.9)
        car.update(row_at(run, time))
        i = row_at(run, time)
        ax.annotate(f"{run.t[i]:.1f} s", (run.x[i], run.y[i]), xytext=(0, -14), textcoords="offset points",
                    fontsize=7, ha="center", color="0.25", zorder=10)
    pad = run.body_length + 2
    ax.update_datalim([(run.x.min() - pad, run.y.min() - pad), (run.x.max() + pad, run.y.max() + pad)])
    ax.autoscale_view()
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{run.path}  [{run.kind}]: the car every {every:g} s", fontsize=11)
    ax.text(0.01, 0.01, legend_text(run, scale), transform=ax.transAxes, fontsize=8, va="bottom",
            bbox=dict(facecolor="white", alpha=0.8, edgecolor="0.8"), zorder=20)
    if has_grip(run):
        sm = matplotlib.cm.ScalarMappable(cmap=cmap, norm=matplotlib.colors.Normalize(0, 100))
        fig.colorbar(sm, ax=ax, fraction=0.03, pad=0.02).set_label("wheel and arrow colour: tyre grip in use (%)")
    return fig


# ----------------------------------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file")
    ap.add_argument("--static", action="store_true", help="the whole path with the car drawn at intervals")
    ap.add_argument("--every", type=float, default=0, metavar="S", help="seconds between cars in the static view")
    ap.add_argument("--at", type=float, metavar="T", help="show (or save) the dashboard at time T")
    ap.add_argument("--save", metavar="PNG", help="write the static view, or the dashboard at --at, to a file")
    ap.add_argument("--gif", metavar="GIF", help="write the dashboard as an animated GIF")
    ap.add_argument("--start", type=float, metavar="T", help="GIF start time (default: the start of the run)")
    ap.add_argument("--end", type=float, metavar="T", help="GIF end time (default: the end of the run)")
    ap.add_argument("--fps", type=float, default=20, help="GIF frames per second (default 20)")
    ap.add_argument("--speed", type=float, default=1, help="GIF playback speed (default 1, real time)")
    ap.add_argument("--window", type=float, default=14, metavar="M", help="width of the view that follows the car (m)")
    ap.add_argument("--no-follow", action="store_true", help="show the whole path in the main view instead")
    args = ap.parse_args()

    if args.save or args.gif:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    run = load(args.file)
    if args.static:
        fig = draw_static(plt, run, args.every)
        if args.save:
            fig.savefig(args.save, dpi=110, bbox_inches="tight")
            print(f"wrote {args.save}")
            return
        plt.show()
        return

    board = Dashboard(plt, run, args.window, not args.no_follow)
    if args.gif:
        from matplotlib.animation import FuncAnimation, PillowWriter
        start = args.start if args.start is not None else run.t[0]
        end = args.end if args.end is not None else run.t[-1]
        times = np.arange(start, end + 1e-9, args.speed / args.fps)
        animation = FuncAnimation(board.fig, lambda k: board.draw(row_at(run, times[k])), frames=len(times))
        animation.save(args.gif, writer=PillowWriter(fps=args.fps), dpi=70)
        print(f"wrote {args.gif} ({len(times)} frames)")
        return
    if args.save:
        board.draw(row_at(run, args.at if args.at is not None else run.t[0]))
        board.fig.savefig(args.save, dpi=100)
        print(f"wrote {args.save}")
        return
    board.interactive()
    if args.at is not None:
        board.slider.set_val(args.at)
    plt.show()


if __name__ == "__main__":
    main()
