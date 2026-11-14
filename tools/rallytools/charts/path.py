"""The static path picture of a run: the path coloured by speed, with the car drawn at intervals."""
from __future__ import annotations

import math

import matplotlib
import numpy as np
from matplotlib.figure import Figure

from ..overhead import STILL_SPEED, force_scale, grip_use, has_grip, row_at, view_of, wheel_state

# Drawing sizes (m) for parts the telemetry does not describe.
TYRE_WIDTH = 0.22
TRAVEL_ARROW = 3.5       # m: length of the direction-of-travel arrow (the speed is in the title)
YAW_ARC_RADIUS = 0.8     # m: the yaw moment's arc around the CG
YAW_ARC_SWEEP = 270      # deg: the arc of the largest yaw moment in the run
COLOURS = {"velocity": "tab:green", "net": "#e67e22", "moment": "#8e44ad"}
STATE_EDGES = {"LOCK": "black", "SPIN": "magenta", "SLIDE": "deepskyblue", "AIR": "0.6"}


def rotate(x, y, angle):
    c, s = math.cos(angle), math.sin(angle)
    return c * x - s * y, s * x + c * y


def rectangle(cx, cy, length, width, angle):
    """Corners of a rectangle centred on (cx, cy), its length along `angle`."""
    a, b = length / 2, width / 2
    pts = [rotate(px, py, angle) for px, py in ((a, b), (a, -b), (-a, -b), (-a, b))]
    return np.array([(cx + px, cy + py) for px, py in pts])


class CarArtist:
    """The car at one row: body, wheels coloured by grip in use, tyre force arrows, velocity arrow, and (with
    `totals`) the sum of the tyre forces at the CG and their yaw moment as an arc around it."""

    def __init__(self, ax, run, scale, cmap, alpha=1.0, labels=False, totals=True):
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
                                        color=COLOURS["velocity"], alpha=alpha, zorder=7)
        ax.add_patch(self.velocity)
        self.body_force = None
        if run.body_force is not None:
            self.body_force = FancyArrowPatch((0, 0), (0, 0), arrowstyle="-|>", mutation_scale=14, lw=2.4,
                                              color="tab:red", alpha=alpha, zorder=8)
            ax.add_patch(self.body_force)
        self.net = None
        if totals and run.net_force is not None:
            self.net = FancyArrowPatch((0, 0), (0, 0), arrowstyle="-|>", mutation_scale=18, lw=3.2,
                                       color=COLOURS["net"], alpha=alpha, zorder=9)
            self.net.set_path_effects(outline)
            ax.add_patch(self.net)
        self.arc = self.arc_head = None
        if totals and run.yaw_moment is not None:
            self.moment_max = float(np.nanmax(np.abs(run.yaw_moment)))
            self.arc, = ax.plot([], [], color=COLOURS["moment"], lw=3.0, alpha=alpha, zorder=9,
                                solid_capstyle="round")
            self.arc_head = Polygon(np.zeros((3, 2)), closed=True, color=COLOURS["moment"], alpha=alpha, zorder=9)
            ax.add_patch(self.arc_head)
        self.show_labels = labels

    def artists(self):
        a = [self.body, self.nose, self.velocity] + self.tyres + self.forces + self.labels
        return a + [x for x in (self.body_force, self.net, self.arc, self.arc_head) if x is not None]

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
            tyre.set_edgecolor(STATE_EDGES.get(state, "black"))
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
        if self.net is not None:
            fx, fy = rotate(r.net_force[0][i], r.net_force[1][i], yaw)
            self._arrow(self.net, cx, cy, fx * self.scale, fy * self.scale)
        if self.arc is not None:
            self._moment(cx, cy, yaw, r.yaw_moment[i])

    def _moment(self, cx, cy, yaw, mz):
        """An arc around the CG, centred on the heading and running the way the moment turns the car; its length
        is the moment's share of the largest in the run."""
        share = abs(mz) / self.moment_max if self.moment_max > 0 else 0.0
        self.arc.set_visible(share > 0.02)
        self.arc_head.set_visible(share > 0.02)
        if share <= 0.02:
            return
        sweep = math.radians(YAW_ARC_SWEEP) * share
        turn = 1.0 if mz > 0 else -1.0
        angles = yaw + turn * np.linspace(-sweep / 2, sweep / 2, 40)
        xs, ys = cx + YAW_ARC_RADIUS * np.cos(angles), cy + YAW_ARC_RADIUS * np.sin(angles)
        self.arc.set_data(xs, ys)
        end = np.array([xs[-1], ys[-1]])
        along = turn * np.array([-math.sin(angles[-1]), math.cos(angles[-1])])  # the way the arc runs at its end
        out = np.array([math.cos(angles[-1]), math.sin(angles[-1])])
        self.arc_head.set_xy(np.array([end + 0.3 * along, end + 0.14 * out, end - 0.14 * out]))


def grip_cmap():
    return matplotlib.colormaps["RdYlGn_r"]


def legend_text(run, scale, totals=True):
    parts = ["green arrow: direction of travel (its angle to the body is the sideslip)"]
    if scale > 0:
        parts.append(f"force arrows: 1 m = {1 / scale:,.0f} N")
    if totals and run.net_force is not None:
        parts.append("orange: sum of the tyre forces; purple arc: their yaw moment")
    if any(w.omega is not None for w in run.wheels):
        parts.append("tyre edge: hatched locked, magenta spinning, blue sliding")
    return "\n".join(parts)



def figure(run, every=0, fig=None) -> Figure:
    """The path coloured by speed with the car drawn every `every` seconds (0: about ten cars), for a Run of any
    kind. Draws into fig when given (an empty figure, such as one from pyplot for a window), else into a new
    Figure. Raises ValueError for a run this view does not know."""
    from matplotlib.collections import LineCollection
    view = view_of(run)
    if fig is None:
        fig = Figure(figsize=(12, 7.5))
    else:
        fig.set_size_inches(12, 7.5)
    ax = fig.subplots()
    points = np.column_stack([view.x, view.y]).reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    path = LineCollection(segments, cmap="viridis", lw=2.5, zorder=2)
    path.set_array(view.speed[:-1])
    ax.add_collection(path)
    fig.colorbar(path, ax=ax, orientation="horizontal", fraction=0.04, pad=0.09, aspect=50).set_label(
        "path colour: speed (m/s)")

    scale = force_scale(view, totals=False)
    cmap = grip_cmap()
    duration = view.t[-1] - view.t[0]
    every = every if every else max(round(duration / 10, 1), 0.1)
    times = np.arange(view.t[0], view.t[-1] + 1e-9, every)
    for time in times:
        car = CarArtist(ax, view, scale, cmap, alpha=0.9, totals=False)
        i = row_at(view, time)
        car.update(i)
        ax.annotate(f"{view.t[i]:.1f} s", (view.x[i], view.y[i]), xytext=(0, -14), textcoords="offset points",
                    fontsize=7, ha="center", color="0.25", zorder=10)
    pad = view.body_length + 2
    ax.update_datalim([(view.x.min() - pad, view.y.min() - pad), (view.x.max() + pad, view.y.max() + pad)])
    ax.autoscale_view()
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"{view.path}  [{view.kind}]: the car every {every:g} s", fontsize=11)
    ax.text(0.01, 0.01, legend_text(view, scale, totals=False), transform=ax.transAxes, fontsize=8, va="bottom",
            bbox=dict(facecolor="white", alpha=0.8, edgecolor="0.8"), zorder=20)
    if has_grip(view):
        sm = matplotlib.cm.ScalarMappable(cmap=cmap, norm=matplotlib.colors.Normalize(0, 100))
        fig.colorbar(sm, ax=ax, fraction=0.03, pad=0.02).set_label("wheel and arrow colour: tyre grip in use (%)")
    return fig
