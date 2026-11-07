# Telemetry schema

Telemetry is a CSV file written by `telemetry_writer` (`core/telemetry.hpp`), one row per physics step
plus one for the final state. The simulator writes it; the tools in `tools/` only read it.

## File layout

```
# schema=2 dt=0.001 units: step[count] time[s] pos.x[m] ...
step,time,pos.x,...
0,0,0,...
```

- Line 1 is a comment: the schema number, the time step `dt` in seconds, optional `key=value` facts about the run
  (see below), and the unit of every column.
- Line 2 is the header: dotted column names, comma separated.
- Every following line is one row of numbers.
- Numbers are written with 17 significant digits, so a value reads back as exactly the same double.
- Lines end with `\n` only, on every platform.

## Facts on line 1

Constant facts that are not columns go on line 1 as `key=value`, between `dt` and `units:`, so a viewer can draw
the car to scale without knowing the C++ defaults. Readers that skip comment lines (pandas with `comment="#"`)
ignore them, and adding keys needs no schema change.

| Key | Unit | Written by | Meaning |
|---|---|---|---|
| `model` | - | bicycle (1), single-track (2), four-wheel (4) | Which model wrote the file |
| `lf`, `lr` | m | all three | CG to front and rear axle |
| `max_steer` | rad | all three | Steer limit |
| `track_front`, `track_rear` | m | single-track, four-wheel | Distance between the contact patches of an axle |
| `cg_height` | m | single-track, four-wheel | CG above the ground |
| `mass` | kg | single-track, four-wheel | Car mass |
| `wheel_radius` | m | four-wheel | Rolling radius |
| `mu_front`, `k_front`, `fz0_front` | -, -, N | four-wheel | Front tyre grip: `mu (1 - k (Fz - Fz0) / Fz0) Fz` |
| `mu_rear`, `k_rear`, `fz0_rear` | -, -, N | four-wheel | Rear tyre grip |
| `drive_split_front` | - | four-wheel | Share of the drive torque on the front axle |

Files written before these keys existed have none; `tools/topdown.py` then draws the default car.

## Columns (rigid body)

| # | Name | Unit | Meaning |
|---|---|---|---|
| 1 | `step` | count | Step index, starting at 0 |
| 2 | `time` | s | `step * dt` |
| 3 | `pos.x` | m | Position, world frame |
| 4 | `pos.y` | m | |
| 5 | `pos.z` | m | |
| 6 | `vel.x` | m/s | Velocity, world frame |
| 7 | `vel.y` | m/s | |
| 8 | `vel.z` | m/s | |
| 9 | `quat.w` | - | Orientation, body to world, Hamilton, w first |
| 10 | `quat.x` | - | |
| 11 | `quat.y` | - | |
| 12 | `quat.z` | - | |
| 13 | `omega.x` | rad/s | Angular velocity, body frame |
| 14 | `omega.y` | rad/s | |
| 15 | `omega.z` | rad/s | |
| 16 | `force.x` | N | Applied force, world frame (gravity not included) |
| 17 | `force.y` | N | |
| 18 | `force.z` | N | |
| 19 | `torque.x` | N m | Applied torque, body frame |
| 20 | `torque.y` | N m | |
| 21 | `torque.z` | N m | |

Axes follow ISO 8855: x forward, y left, z up. All values are SI.
Force and torque in a row are the ones that were applied by the step that starts from that row's state.
The last row has no step after it, so it holds the force and torque the script gives at the final time.

## Columns (kinematic bicycle)

The file layout is the same. Each model has its own set of columns, so read the header to see which one a file has.

| # | Name | Unit | Meaning |
|---|---|---|---|
| 1 | `step` | count | Step index, starting at 0 |
| 2 | `time` | s | `step * dt` |
| 3 | `pos.x` | m | Centre of gravity, world frame |
| 4 | `pos.y` | m | |
| 5 | `yaw` | rad | Heading of the body, counter-clockwise from +x |
| 6 | `speed` | m/s | Speed of the CG, along its direction of travel |
| 7 | `steer` | rad | Front wheel angle the model used (limited to max_steer), positive = left |
| 8 | `accel` | m/s^2 | Acceleration command along the heading |
| 9 | `beta` | rad | Sideslip: angle between the heading and the velocity of the CG |
| 10 | `yaw_rate` | rad/s | Rate of change of `yaw` |
| 11 | `a_lat` | m/s^2 | `speed * yaw_rate`; positive when turning left |

`steer`, `accel`, `beta`, `yaw_rate` and `a_lat` belong to the step that starts from that row's state.
The kinematic model is valid while `|a_lat|` stays below about `0.5 * mu * g` (mu: road friction), so compare the two when reading a run.

## Columns (dynamic single-track)

The file layout is the same. The state is the rigid body's, so the columns are derived from it: the
velocity of the CG in the car's own frame, and the axle slip angles and lateral forces.

| # | Name | Unit | Meaning |
|---|---|---|---|
| 1 | `step` | count | Step index, starting at 0 |
| 2 | `time` | s | `step * dt` |
| 3 | `pos.x` | m | Centre of gravity, world frame |
| 4 | `pos.y` | m | |
| 5 | `yaw` | rad | Heading of the body, counter-clockwise from +x |
| 6 | `speed` | m/s | Length of the velocity of the CG |
| 7 | `steer` | rad | Front wheel angle the model used (limited to max_steer), positive = left |
| 8 | `drive_force` | N | Longitudinal force applied (the speed controller's output when a scenario holds speed) |
| 9 | `u` | m/s | Forward speed of the CG, in the car's frame |
| 10 | `v` | m/s | Sideways speed of the CG, positive to the left |
| 11 | `yaw_rate` | rad/s | Angular velocity about the vertical axis |
| 12 | `beta` | rad | Body sideslip: angle between the heading and the velocity of the CG |
| 13 | `a_lat` | m/s^2 | `speed * yaw_rate`; positive when turning left |
| 14 | `axle.front.slipAngle` | rad | Slip angle of the front axle: wheel heading minus the direction of travel |
| 15 | `axle.rear.slipAngle` | rad | Slip angle of the rear axle |
| 16 | `axle.front.Fy` | N | Lateral force of the front axle, sideways to the wheels |
| 17 | `axle.rear.Fy` | N | Lateral force of the rear axle |
| 18 | `wheel.FL.Fz` | N | Vertical load on the front-left wheel, with load transfer |
| 19 | `wheel.FR.Fz` | N | Front-right wheel |
| 20 | `wheel.RL.Fz` | N | Rear-left wheel |
| 21 | `wheel.RR.Fz` | N | Rear-right wheel |
| 22 | `axle.front.grip` | N | Largest force the front axle can take: the sum of mu(Fz) Fz over its wheels |
| 23 | `axle.rear.grip` | N | Rear axle |
| 24 | `axle.rear.Fx` | N | Longitudinal force the rear axle passed to the road: `drive_force` limited by `axle.rear.grip` |

The four `wheel.*.Fz` columns are the wheel loads with the longitudinal transfer (from the traction-limited
`drive_force`) and the lateral transfer (from the forward speed times `yaw_rate`), shared between the axles by
`front_transfer_share`. They always add up to the weight: when a wheel lifts, the other wheel of its axle carries the
whole axle, and when an axle lifts the other one carries the whole car.

`axle.*.grip` uses those loads with load sensitivity: the friction coefficient of each tyre falls as its load rises,
so moving load between the two wheels of an axle lowers the axle's grip. `axle.front.Fy` is limited by
`axle.front.grip`; `axle.rear.Fy` by what `axle.rear.Fx` leaves of `axle.rear.grip` (friction circle).

Schema 2 renamed the single-track columns `alpha_f`, `alpha_r`, `fy_f`, `fy_r` and `fz_fl` to `fz_rr` to the dotted
names above, so a plot script can build a name from an axle or wheel label (book, section 11.1).

The columns from `steer` on belong to the step that starts from that row's state. The slip angles use a
forward speed of at least `kMinForwardSpeed` (1 m/s), so below that `u` and the slip angles do not
match; the model is not meant for that range. `yaw`, `steer`, `speed` and `beta` are the columns
`tools/topdown.py` needs, so it also plays these files. `beta` and `yaw_rate` have the same names and
meaning as in the bicycle files, which makes the two models easy to overlay.

## Columns (four-wheel car)

The file layout is the same. The first columns match the single-track's where they mean the same thing, so the
two models can be overlaid; the pedals replace `drive_force`, and each wheel adds a block of columns.

| # | Name | Unit | Meaning |
|---|---|---|---|
| 1 | `step` | count | Step index, starting at 0 |
| 2 | `time` | s | `step * dt` |
| 3 | `pos.x` | m | Centre of gravity, world frame |
| 4 | `pos.y` | m | |
| 5 | `yaw` | rad | Heading of the body, counter-clockwise from +x |
| 6 | `speed` | m/s | Length of the velocity of the CG |
| 7 | `steer` | rad | Front wheel angle the model used (limited to max_steer), positive = left; both front wheels |
| 8 | `throttle` | 0-1 | Throttle pedal, as given (the speed controller's output when a scenario holds speed) |
| 9 | `brake` | 0-1 | Brake pedal, as given |
| 10 | `handbrake` | 0-1 | Handbrake, as given |
| 11 | `u` | m/s | Forward speed of the CG, in the car's frame |
| 12 | `v` | m/s | Sideways speed of the CG, positive to the left |
| 13 | `yaw_rate` | rad/s | Angular velocity about the vertical axis |
| 14 | `beta` | rad | Body sideslip: angle between the heading and the velocity of the CG |
| 15 | `a_lat` | m/s^2 | `speed * yaw_rate`; positive when turning left |
| 16-23 | `wheel.FL.*` | | Front-left wheel, the eight fields below |
| 24-31 | `wheel.FR.*` | | Front-right wheel |
| 32-39 | `wheel.RL.*` | | Rear-left wheel |
| 40-47 | `wheel.RR.*` | | Rear-right wheel |

Each wheel block, in this order:

| Field | Unit | Meaning |
|---|---|---|
| `omega` | rad/s | Wheel spin, positive rolling forward |
| `slipRatio` | - | Relaxed slip ratio: positive driving, -1 a locked wheel sliding forward |
| `slipAngle` | rad | Relaxed slip angle (`atan` of the lateral slip): positive when the wheel points left of its path |
| `Fz` | N | Vertical load, with the quasi-static load transfer |
| `Fx` | N | Tyre force along the wheel |
| `Fy` | N | Tyre force across the wheel, positive to its left |
| `driveTorque` | N m | Drive torque the pedals put on this wheel |
| `brakeTorque` | N m | Brake torque on this wheel (a magnitude; it opposes the spin) |

`Fx` and `Fy` are in the wheel's own frame, so on a steered front wheel they are turned by `steer` from the body
axes. As in the other files, the columns from `steer` on belong to the step that starts from that row's state.

## Rules

- Look columns up by name, never by position.
- Never rename or remove a column without changing `kTelemetrySchema`.
- New columns go at the end. Readers that look columns up by name keep working, so this needs no schema change.
- Keep the column names, the units and the row function in the same order. `test_rigid_body_telemetry` checks this.
- Files written before the force and torque columns existed have 15 columns. `tools/topdown.py` needs the force columns;
  `tools/plot_run.py` works with either.