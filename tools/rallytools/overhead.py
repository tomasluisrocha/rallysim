"""A run seen from above: the car's body and wheels, each tyre's force and grip, and the motion around them.

Reads the telemetry only; numpy and pandas only (no drawing). view_of() turns one run into a View whatever the
model: four wheels at their real positions with forces and grip, two wheels on the centreline (single-track, with
axle forces; bicycle, without), or a box with the applied force (rigid body). The car's geometry comes from line 1
of the file; older files get CAR_DEFAULTS.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from rallytools.tyre import grip, tyre_from_meta

# Sizes (m) for parts the telemetry does not describe.
OVERHANG = 0.8            # body beyond each axle
BODY_WIDTH_MARGIN = 0.25  # body wider than the track
STILL_SPEED = 0.3         # m/s: below this the sideslip angle means nothing and is not shown
FORCE_ARROW_MAX = 2.5     # m: the largest tyre force in the run is drawn at most this long, and
NET_ARROW_MAX = 3.5       # m: the largest sum of them at most this long (one scale for every force arrow)
LOCK_SPEED = 0.5          # m/s: a wheel that is stopped while the car moves faster than this is locked
SPIN_SLIP = 0.15          # slip ratio beyond which a wheel counts as spinning (or sliding when negative)

# The car's geometry for files written before it was put on line 1: the default four-wheel car.
CAR_DEFAULTS = {"lf": 1.3, "lr": 1.2, "track_front": 1.5, "track_rear": 1.5, "max_steer": 0.6, "wheel_radius": 0.315,
                "cg_height": 0.5, "springs": 1.0}

COLOURS = {"steer": "tab:blue", "throttle": "tab:green", "brake": "tab:red", "handbrake": "tab:orange"}


def with_car_defaults(meta):
    """Line 1's facts, with CAR_DEFAULTS for the geometry an older file does not give."""
    return {**CAR_DEFAULTS, **meta}


@dataclass
class Channel:
    """One input over the run: its value per row and the range its bar spans. A range from below zero is a signed
    bar, drawn from zero (in `below` under zero, if given)."""
    name: str
    values: np.ndarray
    low: float
    high: float
    fmt: str = "{:+.2f}"
    colour: str = "tab:blue"
    below: str = None

    def share(self, i):
        """Row i's value as a share of the range: 0 to 1, or -1 to 1 for a signed bar."""
        v = float(self.values[i])
        if self.low < 0:
            top = max(-self.low, self.high)
            return float(np.clip(v / top, -1, 1)) if top > 0 else 0.0
        span = self.high - self.low
        return float(np.clip((v - self.low) / span, 0, 1)) if span > 0 else 0.0

    def text(self, i):
        return self.fmt.format(self.values[i])


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
class View:
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
    steer_input: Channel = None              # the horizontal input bar
    inputs: list = field(default_factory=list)   # Channel: the vertical input bars
    body_force: tuple = None      # (fx, fy) world, for rigid bodies
    net_force: tuple = None       # (along, across) body frame: the sum of the tyre forces (N)
    yaw_moment: np.ndarray = None  # their moment about the CG (N m), positive turning left
    accel: tuple = None           # (along, across) body frame (m/s^2), for the g-g diagram
    accel_source: str = ""


def body_acceleration(t, vx, vy, yaw):
    """Change of the CG velocity over time in the body's axes: (forward, left), m/s^2."""
    if len(t) < 2:
        return np.zeros_like(t), np.zeros_like(t)
    ax, ay = np.gradient(vx, t), np.gradient(vy, t)
    return ax * np.cos(yaw) + ay * np.sin(yaw), -ax * np.sin(yaw) + ay * np.cos(yaw)


def tyre_totals(wheels):
    """The sum of the tyre forces in the body frame, (along, across) in N, and their moment about the CG in N m
    (positive turning left): each wheel's force turned by its steer, acting at its contact patch."""
    along = across = moment = 0.0
    for w in wheels:
        c, s = np.cos(w.steer), np.sin(w.steer)
        bx, by = w.fx * c - w.fy * s, w.fx * s + w.fy * c
        along, across, moment = along + bx, across + by, moment + w.x * by - w.y * bx
    return (along, across), moment


def sideslip(beta, speed):
    """The sideslip angle, blank (NaN) where the car is (almost) still."""
    return np.where(speed > STILL_SPEED, beta, np.nan)


def symmetric(values, floor):
    m = float(np.nanmax(np.abs(values))) if len(values) else 0.0
    return max(m, floor)


def view_of(run):
    """The View of a rallytools Run. Raises ValueError for a run with neither quat.* nor yaw and speed."""
    df = run.data
    meta = with_car_defaults(run.meta)  # the tyres' own defaults come from rallytools.tyre.tyre_from_meta
    cols = set(df.columns)
    t = df["time"].to_numpy() if "time" in cols else np.arange(len(df), dtype=float)

    if "quat.w" in cols:
        return _rigid_body(run.name, df, t)
    if "yaw" not in cols or "speed" not in cols:
        raise ValueError(f"{run.name}: not a run this view knows (no quat.* or yaw/speed columns)")

    yaw = df["yaw"].to_numpy()
    speed = df["speed"].to_numpy()
    beta = df["beta"].to_numpy() if "beta" in cols else np.zeros_like(yaw)
    vx, vy = speed * np.cos(yaw + beta), speed * np.sin(yaw + beta)
    lf, lr = meta["lf"], meta["lr"]
    tf, tr = meta["track_front"], meta["track_rear"]
    view = View(path=run.name, kind="", t=t, x=df["pos.x"].to_numpy(), y=df["pos.y"].to_numpy(), yaw=yaw,
                vx=vx, vy=vy, speed=speed, beta=sideslip(beta, speed), yaw_rate=df["yaw_rate"].to_numpy(),
                a_lat=body_acceleration(t, vx, vy, yaw)[1],
                body_length=lf + lr + 2 * OVERHANG, body_width=max(tf, tr) + BODY_WIDTH_MARGIN,
                body_front=lf + OVERHANG, tyre_length=2 * meta["wheel_radius"])
    steer = df["steer"].to_numpy()
    zero = np.zeros_like(steer)
    max_steer = math.degrees(meta["max_steer"])
    view.steer_input = Channel("steer", np.degrees(steer), -max_steer, max_steer, "{:+.1f}°", COLOURS["steer"])

    if "wheel.FL.Fx" in cols:
        view.kind = "four-wheel car"
        positions = {"FL": (lf, tf / 2), "FR": (lf, -tf / 2), "RL": (-lr, tr / 2), "RR": (-lr, -tr / 2)}
        for label, (x, y) in positions.items():
            end = "front" if label[0] == "F" else "rear"
            fz = df[f"wheel.{label}.Fz"].to_numpy()
            # logged since schema 5; worked out from the load for older files
            g = (df[f"wheel.{label}.grip"].to_numpy() if f"wheel.{label}.grip" in cols
                 else grip(tyre_from_meta(meta, end), fz))
            view.wheels.append(Wheel(label, x, y, steer if end == "front" else zero,
                                     df[f"wheel.{label}.Fx"].to_numpy(), df[f"wheel.{label}.Fy"].to_numpy(),
                                     fz, g, df[f"wheel.{label}.omega"].to_numpy(),
                                     df[f"wheel.{label}.slipRatio"].to_numpy()))
        view.inputs = [Channel(p, df[p].to_numpy(), 0, 1, "{:.0%}", COLOURS[p])
                       for p in ("throttle", "brake", "handbrake")]
    elif "axle.front.Fy" in cols:
        view.kind = "single-track"
        front_fz = (df["wheel.FL.Fz"] + df["wheel.FR.Fz"]).to_numpy() if "wheel.FL.Fz" in cols else None
        rear_fz = (df["wheel.RL.Fz"] + df["wheel.RR.Fz"]).to_numpy() if "wheel.RL.Fz" in cols else None
        rear_fx = df["axle.rear.Fx"].to_numpy() if "axle.rear.Fx" in cols else zero
        view.wheels = [
            Wheel("front", lf, 0.0, steer, zero, df["axle.front.Fy"].to_numpy(), front_fz,
                  df["axle.front.grip"].to_numpy() if "axle.front.grip" in cols else None),
            Wheel("rear", -lr, 0.0, zero, rear_fx, df["axle.rear.Fy"].to_numpy(), rear_fz,
                  df["axle.rear.grip"].to_numpy() if "axle.rear.grip" in cols else None)]
        drive, top = df["drive_force"].to_numpy(), symmetric(df["drive_force"].to_numpy(), 1.0)
        view.inputs = [Channel("drive force", drive, -top, top, "{:+.0f} N", COLOURS["throttle"], COLOURS["brake"])]
    else:
        view.kind = "kinematic bicycle"
        view.wheels = [Wheel("front", lf, 0.0, steer), Wheel("rear", -lr, 0.0, zero)]
        if "accel" in cols:
            accel, top = df["accel"].to_numpy(), symmetric(df["accel"].to_numpy(), 0.1)
            view.inputs = [Channel("accel", accel, -top, top, "{:+.2f} m/s²", COLOURS["throttle"],
                                   COLOURS["brake"])]

    if view.wheels[0].fx is not None:
        if "yaw_moment" in cols:  # logged since schema 5
            view.net_force = (df["net.Fx"].to_numpy(), df["net.Fy"].to_numpy())
            view.yaw_moment = df["yaw_moment"].to_numpy()
        else:
            view.net_force, view.yaw_moment = tyre_totals(view.wheels)
    if view.net_force is not None and meta.get("mass", 0) > 0:
        view.accel = (view.net_force[0] / meta["mass"], view.net_force[1] / meta["mass"])
        view.accel_source = "tyre forces / mass"
    else:
        view.accel, view.accel_source = body_acceleration(t, vx, vy, yaw), "change of velocity"
    return view


def _rigid_body(name, df, t):
    w, x, y, z = (df[f"quat.{c}"].to_numpy() for c in "wxyz")
    yaw = np.arctan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z))
    vx, vy = df["vel.x"].to_numpy(), df["vel.y"].to_numpy()
    forward = np.cos(yaw) * vx + np.sin(yaw) * vy
    left = -np.sin(yaw) * vx + np.cos(yaw) * vy
    speed = np.hypot(vx, vy)
    fx = df["force.x"].to_numpy() if "force.x" in df.columns else np.zeros_like(t)
    fy = df["force.y"].to_numpy() if "force.y" in df.columns else np.zeros_like(t)
    accel = body_acceleration(t, vx, vy, yaw)
    view = View(path=name, kind="rigid body", t=t, x=df["pos.x"].to_numpy(), y=df["pos.y"].to_numpy(), yaw=yaw,
                vx=vx, vy=vy, speed=speed, beta=sideslip(np.arctan2(left, forward), speed),
                yaw_rate=df["omega.z"].to_numpy(), a_lat=accel[1],
                body_length=4.0, body_width=1.8, body_front=2.0, tyre_length=0.0, body_force=(fx, fy),
                accel=accel, accel_source="change of velocity")
    view.inputs = [Channel("force x", fx, -symmetric(fx, 1.0), symmetric(fx, 1.0), "{:+.0f} N"),
                   Channel("force y", fy, -symmetric(fy, 1.0), symmetric(fy, 1.0), "{:+.0f} N")]
    if "torque.z" in df.columns:
        tz = df["torque.z"].to_numpy()
        view.inputs.append(Channel("torque z", tz, -symmetric(tz, 1.0), symmetric(tz, 1.0), "{:+.0f} N m"))
    return view


def force_scale(view, totals=True):
    """Metres of arrow per newton, one scale for every force arrow: the largest tyre (or applied) force in the run
    is at most FORCE_ARROW_MAX long and, with `totals` drawn, the largest sum of the tyre forces at most
    NET_ARROW_MAX."""
    forces = [(w.fx, w.fy) for w in view.wheels if w.fx is not None]
    forces += [view.body_force] if view.body_force is not None else []
    largest = max((float(np.nanmax(np.hypot(*f))) for f in forces), default=0.0)
    scales = [FORCE_ARROW_MAX / largest] if largest > 0 else []
    net = float(np.nanmax(np.hypot(*view.net_force))) if totals and view.net_force is not None else 0.0
    if net > 0:
        scales.append(NET_ARROW_MAX / net)
    return min(scales) if scales else 0.0


def has_grip(view):
    return any(wh.grip is not None and wh.fx is not None for wh in view.wheels)


def grip_use(wheel, i):
    """Share of the tyre's grip in use, 0..1, or None if the file does not say."""
    if wheel.grip is None or wheel.fx is None:
        return None
    g = wheel.grip[i]
    return min(float(np.hypot(wheel.fx[i], wheel.fy[i]) / g), 1.0) if g > 0 else 1.0


def wheel_state(view, wheel, i):
    """'AIR', 'LOCK', 'SPIN', 'SLIDE' or '' for a wheel at row i."""
    if wheel.fz is not None and wheel.fz[i] <= 0:
        return "AIR"
    if wheel.omega is not None and wheel.omega[i] == 0 and abs(view.speed[i]) > LOCK_SPEED:
        return "LOCK"
    # slip means nothing when nothing rolls: at a standstill (omega R - vx) / max(|vx|, |omega R|) is 0/0 and reads +-1
    rolling = max(abs(view.speed[i]), abs(wheel.omega[i]) * view.tyre_length / 2 if wheel.omega is not None else 0.0)
    if wheel.slip_ratio is not None and rolling > LOCK_SPEED:
        if wheel.slip_ratio[i] > SPIN_SLIP:
            return "SPIN"
        if wheel.slip_ratio[i] < -SPIN_SLIP:
            return "SLIDE"
    return ""


def row_at(view, time):
    return int(np.clip(np.searchsorted(view.t, time), 0, len(view.t) - 1))
