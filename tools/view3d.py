"""3D view of a four-wheel run: the car at its real pose, and every force drawn where it acts.

Reads the telemetry only (like topdown.py); it never touches the physics.

  body       a translucent box at the run's position, heading, roll, pitch and ride height. Roll, pitch and the
             ride height's change from the start are exaggerated (--exaggerate, default 4) because 1 degree is
             invisible at true scale; the HUD gives the true angles.
  wheels     at their contact patches, turned by the steer, with a spoke that turns with the wheel's spin (a
             locked wheel's spoke stops), coloured by how much grip the tyre uses; a strut from the body to each
             wheel, coloured by how loaded its corner is.
  at each contact patch (one force scale for every arrow, shown on screen):
             blue, straight up   the vertical load Fz: load transfer is the change in these
             red, on the ground  the tyre force (Fx, Fy), turned with the wheel
             grey circle         the grip mu(Fz) Fz: the red arrow's length against it is the grip in use
  at the CG  green              velocity (0.15 s of travel)
             orange             the sum of the four tyre forces, what pushes the car around (same force scale)
  ground     a grid that moves with the car, and the path behind it

Usage:
  py tools/view3d.py out/runs/fw_handbrake.csv                       play, pause, scrub, turn the view
  py tools/view3d.py out/runs/fw_brake.csv --at 1.5 --save out/scratch/brake3d.png
  py tools/view3d.py out/runs/fw_circle.csv --gif out/gif/fw_circle_3d.gif --start 1 --end 6
  py tools/view3d.py out/runs/fw_lane.csv --chase                    camera behind the car, turning with it
  py tools/view3d.py out/runs/fw_brake.csv --exaggerate 1             true-scale body motion
  py tools/view3d.py out/runs/fw_lane.csv --perspective               a perspective camera instead of isometric

The default camera is isometric: a parallel (orthographic) projection from 35.26 deg up, 45 deg round, so every
arrow and wheel is drawn at its true length wherever it sits in the picture.

Keys: space play/pause, left/right 0.1 s (shift: 1 s), up/down playback speed x2 / /2, c chase camera on/off,
p isometric / perspective, home/end. Drag with the mouse to turn the view (while the chase camera is off).
"""
import argparse
import math
import sys
from pathlib import Path

import matplotlib
import numpy as np

import simdata

OVERHANG = 0.8           # m: body beyond each axle (as topdown.py)
BODY_WIDTH_MARGIN = 0.25  # m: body wider than the track
BODY_TOP = 0.75          # m: roof above the CG (the underside is just above the wheel centres)
STRUT_LENGTH = 0.35      # m: drawn length of a strut at rest, from the wheel centre up to the body
VELOCITY_TIME = 0.15     # s: the velocity arrow is this much travel
FORCE_ARROW = 1.2        # m: the largest vertical load in the run is drawn this long; every force uses that scale
TRAIL_TIME = 4.0         # s of path drawn behind the car
LOAD_OFFSET = 0.28       # m: the load arrow stands this far outboard of its wheel
ZOOM = 1.6               # how far the 3D box is zoomed in
SLIDING_SLIP = 0.3       # |slip ratio| beyond which the HUD calls a wheel sliding (or spinning)
LOCK_SPEED = 0.5         # m/s: a stopped wheel on a car moving faster than this is locked
ISO_ELEV = math.degrees(math.atan(1 / math.sqrt(2)))  # 35.26 deg: with an orthographic projection, a true isometric view
ISO_AZIM = -45.0
COLOURS = {"load": "#2f6fd6", "tyre": "#d62f2f", "grip": "#888888", "velocity": "#1f9d55", "net": "#e8871e"}


# ----------------------------------------------------------------------------------------------------------
# Reading
# ----------------------------------------------------------------------------------------------------------

class Run:
    """What the view needs from a four-wheel telemetry file, as arrays over the rows."""

    def __init__(self, path, exaggerate):
        data, meta = simdata.read_run(path)
        if not simdata.is_four_wheel(data):
            sys.exit(f"{path}: not a four-wheel run (view3d needs the wheel.* columns)")
        missing = [k for k in ("roll", "pitch", "pos.z") if k not in data.columns]
        if missing:
            sys.exit(f"{path}: written before schema 3 (no {', '.join(missing)}); rerun the scenario")
        self.path = path
        self.name = simdata.label_of(path)
        self.meta = meta
        col = lambda name: data[name].to_numpy(dtype=float)
        self.t = col("time")
        self.dt = float(np.median(np.diff(self.t))) if len(self.t) > 1 else 0.001
        self.x, self.y, self.yaw = col("pos.x"), col("pos.y"), np.unwrap(col("yaw"))
        self.speed, self.u, self.v = col("speed"), col("u"), col("v")
        self.steer = col("steer")
        self.inputs = {k: col(k) for k in ("steer", "throttle", "brake", "handbrake")}
        self.a_lat = col("a_lat")

        # body pose, exaggerated about where it started
        self.exaggerate = exaggerate
        self.roll_true, self.pitch_true, self.z_true = col("roll"), col("pitch"), col("pos.z")
        self.springs = meta.get("springs", 1.0) > 0.5
        self.cg_height = meta.get("cg_height", 0.5)
        z0 = self.z_true[0] if self.springs else self.cg_height
        z_true = self.z_true if self.springs else np.full_like(self.t, self.cg_height)  # planar: CG stays put
        self.z = z0 + exaggerate * (z_true - z0)
        self.roll, self.pitch = exaggerate * self.roll_true, exaggerate * self.pitch_true

        # geometry (line 1, with topdown's defaults for older files)
        lf, lr = meta.get("lf", 1.3), meta.get("lr", 1.2)
        tf, tr = meta.get("track_front", 1.5), meta.get("track_rear", 1.5)
        self.radius = meta.get("wheel_radius", 0.315)
        self.body = (lf + OVERHANG, -(lr + OVERHANG), 0.5 * max(tf, tr) + BODY_WIDTH_MARGIN)
        self.wheel_xy = {"FL": (lf, tf / 2), "FR": (lf, -tf / 2), "RL": (-lr, tr / 2), "RR": (-lr, -tr / 2)}

        # per wheel: forces, grip, spin angle
        self.wheels = {}
        for w in simdata.WHEELS:
            tyre = simdata.tyre_from_meta(meta, "front" if w[0] == "F" else "rear")
            fz = col(f"wheel.{w}.Fz")
            omega = col(f"wheel.{w}.omega")
            self.wheels[w] = {
                "fx": col(f"wheel.{w}.Fx"), "fy": col(f"wheel.{w}.Fy"), "fz": fz, "omega": omega,
                "slip_ratio": col(f"wheel.{w}.slipRatio"),
                "grip": simdata.grip(tyre, fz),
                "spin": np.concatenate([[0.0], np.cumsum(omega[:-1] * np.diff(self.t))]),  # rad turned so far
                "static": fz[0] if fz[0] > 0 else np.nan,
            }
        largest_load = max(float(np.max(w["fz"])) for w in self.wheels.values())
        self.force_scale = FORCE_ARROW / largest_load if largest_load > 0 else 1e-3  # m per N

    def index(self, time):
        return int(np.clip(np.searchsorted(self.t, time), 0, len(self.t) - 1))


# ----------------------------------------------------------------------------------------------------------
# Geometry
# ----------------------------------------------------------------------------------------------------------

def rotation(yaw, pitch, roll):
    """Body to world, yaw-pitch-roll (Rz Ry Rx) with ISO 8855 signs, as roll_angle/pitch_angle in rigid_body.hpp."""
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def body_faces(run, i, rot, cg):
    """The six faces of the body box, world coordinates."""
    front, rear, half = run.body
    bottom = -run.cg_height + run.radius + 0.05  # just above the wheel centres at rest
    corners = np.array([[x, y, z] for x in (rear, front) for y in (-half, half) for z in (bottom, BODY_TOP)])
    world = corners @ rot.T + cg
    idx = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [world[list(f)] for f in idx]


def wheel_heading(run, i, label):
    return run.yaw[i] + (run.steer[i] if label[0] == "F" else 0.0)


def grip_colour(use):
    """Green (little grip used) through yellow to red (at the limit)."""
    import matplotlib.cm as cm
    return cm.RdYlGn_r(float(np.clip(use, 0, 1)) * 0.95)


def darker(colour, amount=0.35):
    """The same colour mixed with black, for text on white (the yellow middle of grip_colour is hard to read)."""
    r, g, b = matplotlib.colors.to_rgb(colour)
    return (r * (1 - amount), g * (1 - amount), b * (1 - amount))


def load_colour(fz, static):
    """Blue below the corner's load at the start, red above."""
    import matplotlib.cm as cm
    if not np.isfinite(static) or static <= 0:
        return "#555555"
    return cm.coolwarm(float(np.clip(0.5 + 0.5 * (fz / static - 1), 0, 1)))


# ----------------------------------------------------------------------------------------------------------
# Drawing one frame
# ----------------------------------------------------------------------------------------------------------

class Scene:
    def __init__(self, plt, run, window, chase, isometric):
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        self.plt, self.run, self.window, self.chase, self.isometric = plt, run, window, chase, isometric
        self.Poly3DCollection = Poly3DCollection
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_axes([0.0, 0.08, 0.78, 0.92], projection="3d")
        self.hud = self.fig.add_axes([0.78, 0.08, 0.22, 0.92])
        self.hud.axis("off")
        self.elev, self.azim = 22.0, -60.0
        self.i = 0

    def draw(self, i):
        run, ax = self.run, self.ax
        self.i = i
        if not self.chase:
            self.elev, self.azim = ax.elev, ax.azim  # keep the view the user turned to
        ax.cla()
        cg = np.array([run.x[i], run.y[i], run.z[i]])
        rot = rotation(run.yaw[i], run.pitch[i], run.roll[i])
        k = run.force_scale

        self.ground(cg)
        start = max(0, run.index(run.t[i] - TRAIL_TIME))
        ax.plot(run.x[start:i + 1], run.y[start:i + 1], np.zeros(i + 1 - start), color="#999999", lw=1)

        # body
        body = self.Poly3DCollection(body_faces(run, i, rot, cg), facecolor=(0.55, 0.62, 0.75, 0.18),
                                     edgecolor=(0.25, 0.3, 0.4, 0.8), linewidths=0.8)
        ax.add_collection3d(body)
        ax.scatter(*cg, color="black", s=12)

        net = np.zeros(3)
        for label, (wx, wy) in run.wheel_xy.items():
            w = run.wheels[label]
            heading = wheel_heading(run, i, label)
            fwd = np.array([math.cos(heading), math.sin(heading), 0.0])
            left = np.array([-math.sin(heading), math.cos(heading), 0.0])
            # the strut's top moves with the body; the wheel stays on the ground below it
            top = cg + rot @ np.array([wx, wy, -run.cg_height + run.radius + STRUT_LENGTH])
            patch = np.array([top[0], top[1], 0.0])
            centre = patch + np.array([0, 0, run.radius])
            fz, fx, fy, grip = w["fz"][i], w["fx"][i], w["fy"][i], w["grip"][i]
            use = math.hypot(fx, fy) / grip if grip > 0 else 0.0

            ax.plot(*zip(top, centre), color=load_colour(fz, w["static"]), lw=3)           # strut
            self.wheel(centre, fwd, run.radius, w["spin"][i], grip_colour(use) if fz > 0 else "#bbbbbb")
            if fz > 0:
                # the load arrow stands just outboard of the wheel, so it does not hide in the strut
                outboard = left * (LOAD_OFFSET if wy > 0 else -LOAD_OFFSET)
                ax.quiver(*(patch + outboard), 0, 0, fz * k, color=COLOURS["load"], lw=2, arrow_length_ratio=0.12)
                force = (fx * fwd + fy * left) * k
                ax.quiver(*(patch + [0, 0, 0.01]), *force, color=COLOURS["tyre"], lw=2.2, arrow_length_ratio=0.15)
                theta = np.linspace(0, 2 * np.pi, 49)
                ax.plot(patch[0] + grip * k * np.cos(theta), patch[1] + grip * k * np.sin(theta),
                        np.full_like(theta, 0.005), color=COLOURS["grip"], lw=0.8)
                net += fx * fwd + fy * left

        velocity = np.array([run.u[i] * math.cos(run.yaw[i]) - run.v[i] * math.sin(run.yaw[i]),
                             run.u[i] * math.sin(run.yaw[i]) + run.v[i] * math.cos(run.yaw[i]), 0.0])
        ax.quiver(*cg, *(velocity * VELOCITY_TIME), color=COLOURS["velocity"], lw=2, arrow_length_ratio=0.1)
        ax.quiver(*cg, *(net * k), color=COLOURS["net"], lw=2.5, arrow_length_ratio=0.12)

        self.frame(cg)
        self.write_hud(i)

    def ground(self, cg):
        half, step = self.window / 2, 2.0
        lo_x, hi_x = math.floor((cg[0] - half) / step) * step, math.ceil((cg[0] + half) / step) * step
        lo_y, hi_y = math.floor((cg[1] - half) / step) * step, math.ceil((cg[1] + half) / step) * step
        for gx in np.arange(lo_x, hi_x + step / 2, step):
            self.ax.plot([gx, gx], [lo_y, hi_y], [0, 0], color="#dddddd", lw=0.6)
        for gy in np.arange(lo_y, hi_y + step / 2, step):
            self.ax.plot([lo_x, hi_x], [gy, gy], [0, 0], color="#dddddd", lw=0.6)

    def wheel(self, centre, fwd, radius, spin, colour):
        up = np.array([0.0, 0.0, 1.0])
        a = np.linspace(0, 2 * np.pi, 33)
        ring = centre + radius * (np.outer(np.cos(a), fwd) + np.outer(np.sin(a), up))
        self.ax.plot(ring[:, 0], ring[:, 1], ring[:, 2], color=colour, lw=3)
        # rolling forward turns the top of the wheel forward: the spoke's angle in (fwd, up) falls as it spins
        angle = math.pi / 2 - spin
        tip = centre + radius * (math.cos(angle) * fwd + math.sin(angle) * up)
        self.ax.plot(*zip(centre, tip), color="black", lw=1.5)

    def frame(self, cg):
        ax, half = self.ax, self.window / 2
        ax.set_xlim(cg[0] - half, cg[0] + half)
        ax.set_ylim(cg[1] - half, cg[1] + half)
        ax.set_zlim(0, self.window / 3)
        ax.set_box_aspect((1, 1, 1 / 3), zoom=ZOOM)  # equal scale on every axis
        ax.set_axis_off()  # the ground grid (2 m squares) and the HUD's force scale stand in for axes
        # isometric: parallel projection, so every arrow and wheel keeps its true length wherever it is in the picture
        ax.set_proj_type("ortho" if self.isometric else "persp")
        if self.chase:
            # behind the car, looking along its heading; isometric keeps its 45 degrees, now to the car's side
            self.azim = math.degrees(self.run.yaw[self.i]) + (180 + ISO_AZIM + 45 if self.isometric else 180)
        ax.view_init(elev=self.elev, azim=self.azim)

    def write_hud(self, i):
        run, h = self.run, self.hud
        h.cla()
        h.axis("off")
        newton_per_m = 1 / run.force_scale
        lines = [
            (f"{run.name}", "black", 12, "bold"),
            (f"t = {run.t[i]:.2f} s", "black", 11, "normal"),
            (f"speed {run.speed[i]:.1f} m/s ({run.speed[i] * 3.6:.0f} km/h)", "black", 10, "normal"),
            (f"roll {math.degrees(run.roll_true[i]):+.2f} deg  (+ right side down)", "black", 10, "normal"),
            (f"pitch {math.degrees(run.pitch_true[i]):+.2f} deg  (+ nose down)", "black", 10, "normal"),
            (f"body motion drawn x{run.exaggerate:g}", "#666666", 9, "italic"),
            ("", "black", 6, "normal"),
            (f"steer {math.degrees(run.inputs['steer'][i]):+.1f} deg", "black", 10, "normal"),
            (f"throttle {run.inputs['throttle'][i]:.2f}   brake {run.inputs['brake'][i]:.2f}", "black", 10, "normal"),
            (f"handbrake {run.inputs['handbrake'][i]:.2f}", "black", 10, "normal"),
            ("", "black", 6, "normal"),
        ]
        for label in simdata.WHEELS:
            w = run.wheels[label]
            use = math.hypot(w["fx"][i], w["fy"][i]) / w["grip"][i] if w["grip"][i] > 0 else 0.0
            lines.append((f"{label}  Fz {w['fz'][i]:5.0f} N  {use * 100:3.0f}% {self.wheel_state(label, i)}",
                          darker(grip_colour(use)) if w["fz"][i] > 0 else "#999999", 10, "normal"))
        lines += [
            ("% of grip in use; past the peak a sliding", "#666666", 9, "italic"),
            ("tyre gives less than 100%", "#666666", 9, "italic"),
            ("", "black", 6, "normal"),
            (f"forces: 1 m = {newton_per_m / 1000:.1f} kN", "black", 10, "bold"),
            ("blue  vertical load Fz", COLOURS["load"], 10, "normal"),
            ("red   tyre force (Fx, Fy)", COLOURS["tyre"], 10, "normal"),
            ("grey  grip circle mu(Fz) Fz", COLOURS["grip"], 10, "normal"),
            ("orange  sum of the tyre forces", COLOURS["net"], 10, "normal"),
            (f"green  velocity ({VELOCITY_TIME:g} s of travel)", COLOURS["velocity"], 10, "normal"),
            ("struts: blue lighter, red heavier", "#666666", 9, "italic"),
            ("than at the start", "#666666", 9, "italic"),
        ]
        y = 0.97
        for text, colour, size, style in lines:
            h.text(0.02, y, text, color=colour, fontsize=size, fontweight="bold" if style == "bold" else "normal",
                   fontstyle="italic" if style == "italic" else "normal", transform=h.transAxes, va="top",
                   family="monospace" if text[:2] in ("FL", "FR", "RL", "RR") else None)
            y -= 0.034 if size >= 10 else 0.026

    def wheel_state(self, label, i):
        """'locked', 'spinning', 'sliding', 'in the air' or '' (rolling), for the HUD."""
        run, w = self.run, self.run.wheels[label]
        if w["fz"][i] <= 0:
            return "in the air"
        if run.speed[i] > LOCK_SPEED and w["omega"][i] == 0:
            return "locked"
        if run.speed[i] > LOCK_SPEED and w["slip_ratio"][i] > SLIDING_SLIP:
            return "spinning"
        if run.speed[i] > LOCK_SPEED and w["slip_ratio"][i] < -SLIDING_SLIP:
            return "sliding"
        return ""

    # ------------------------------------------------------------------------------------------------------
    # Playing
    # ------------------------------------------------------------------------------------------------------

    def interactive(self):
        from matplotlib.widgets import Slider
        run = self.run
        slider_ax = self.fig.add_axes([0.08, 0.02, 0.62, 0.03])
        self.slider = Slider(slider_ax, "t [s]", run.t[0], run.t[-1], valinit=run.t[0])
        self.slider.on_changed(lambda value: self.show(run.index(value)))
        self.playing, self.speed = False, 1.0
        self.timer = self.fig.canvas.new_timer(interval=60)
        self.timer.add_callback(self.tick)
        self.timer.start()
        self.fig.canvas.mpl_connect("key_press_event", self.key)
        self.show(0)

    def show(self, i):
        self.draw(i)
        self.fig.canvas.draw_idle()

    def tick(self):
        if not self.playing:
            return
        run = self.run
        time = run.t[self.i] + self.speed * 0.06
        if time > run.t[-1]:
            time, self.playing = run.t[-1], False
        self.slider.set_val(time)

    def key(self, event):
        run = self.run
        jumps = {"left": -0.1, "right": 0.1, "shift+left": -1.0, "shift+right": 1.0}
        if event.key == " ":
            self.playing = not self.playing
        elif event.key in jumps:
            self.slider.set_val(float(np.clip(run.t[self.i] + jumps[event.key], run.t[0], run.t[-1])))
        elif event.key == "up":
            self.speed *= 2
        elif event.key == "down":
            self.speed /= 2
        elif event.key == "home":
            self.slider.set_val(run.t[0])
        elif event.key == "end":
            self.slider.set_val(run.t[-1])
        elif event.key == "c":
            self.chase = not self.chase
            self.show(self.i)
        elif event.key == "p":
            self.isometric = not self.isometric
            if self.isometric:
                self.elev, self.azim = ISO_ELEV, ISO_AZIM
            self.show(self.i)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("file", help="a four-wheel telemetry CSV, e.g. out/runs/fw_handbrake.csv")
    ap.add_argument("--at", type=float, metavar="T", help="show (or save) the scene at time T")
    ap.add_argument("--save", metavar="PNG", help="write the scene at --at (default: the start) to a file")
    ap.add_argument("--gif", metavar="GIF", help="write the run as an animated GIF")
    ap.add_argument("--start", type=float, metavar="T", help="GIF start time (default: the start of the run)")
    ap.add_argument("--end", type=float, metavar="T", help="GIF end time (default: the end of the run)")
    ap.add_argument("--fps", type=float, default=15, help="GIF frames per second (default 15)")
    ap.add_argument("--speed", type=float, default=1, help="GIF playback speed (default 1, real time)")
    ap.add_argument("--window", type=float, default=10, metavar="M", help="width of the view around the car (m)")
    ap.add_argument("--exaggerate", type=float, default=4, metavar="K", help="roll, pitch and ride height drawn K times (default 4)")
    ap.add_argument("--chase", action="store_true", help="camera behind the car, turning with it")
    ap.add_argument("--perspective", action="store_true",
                    help="a perspective camera (default: isometric, a parallel projection at 35.26 / -45 deg)")
    ap.add_argument("--elev", type=float, help="camera elevation (deg; default 35.26 isometric, 22 perspective)")
    ap.add_argument("--azim", type=float, help="camera azimuth (deg) when not chasing (default -45 isometric, -60)")
    args = ap.parse_args()

    if args.save or args.gif:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    run = Run(args.file, args.exaggerate)
    isometric = not args.perspective
    scene = Scene(plt, run, args.window, args.chase, isometric)
    scene.elev = args.elev if args.elev is not None else (ISO_ELEV if isometric else 22.0)
    scene.azim = args.azim if args.azim is not None else (ISO_AZIM if isometric else -60.0)

    if args.gif:
        from matplotlib.animation import FuncAnimation, PillowWriter
        start = args.start if args.start is not None else run.t[0]
        end = args.end if args.end is not None else run.t[-1]
        times = np.arange(start, end + 1e-9, args.speed / args.fps)
        Path(args.gif).parent.mkdir(parents=True, exist_ok=True)
        animation = FuncAnimation(scene.fig, lambda n: scene.draw(run.index(times[n])), frames=len(times))
        animation.save(args.gif, writer=PillowWriter(fps=args.fps), dpi=70)
        print(f"wrote {args.gif} ({len(times)} frames)")
        return
    if args.save:
        scene.draw(run.index(args.at if args.at is not None else run.t[0]))
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        scene.fig.savefig(args.save, dpi=100)
        print(f"wrote {args.save}")
        return
    scene.interactive()
    if args.at is not None:
        scene.slider.set_val(args.at)
    plt.show()


if __name__ == "__main__":
    main()
