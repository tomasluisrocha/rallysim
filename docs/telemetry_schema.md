# Telemetry schema

Telemetry is a CSV file written by `telemetry_writer` (`core/telemetry.hpp`), one row per physics step
plus one for the final state. The simulator writes it; the tools in `tools/` only read it.

## File layout

```
# schema=1 dt=0.001 units: step[count] time[s] pos.x[m] ...
step,time,pos.x,...
0,0,0,...
```

- Line 1 is a comment: the schema number, the time step `dt` in seconds, and the unit of every column.
- Line 2 is the header: dotted column names, comma separated.
- Every following line is one row of numbers.
- Numbers are written with 17 significant digits, so a value reads back as exactly the same double.
- Lines end with `\n` only, on every platform.

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
| 14 | `alpha_f` | rad | Slip angle of the front axle |
| 15 | `alpha_r` | rad | Slip angle of the rear axle |
| 16 | `fy_f` | N | Lateral force of the front axle, sideways to the wheels |
| 17 | `fy_r` | N | Lateral force of the rear axle |

The columns from `steer` on belong to the step that starts from that row's state. The slip angles use a
forward speed of at least `kMinForwardSpeed` (1 m/s), so below that `u` and the slip angles do not
match; the model is not meant for that range. `yaw`, `steer`, `speed` and `a_lat` are the columns
`tools/topdown.py` needs, so it also plays these files. `beta` and `yaw_rate` have the same names and
meaning as in the bicycle files, which makes the two models easy to overlay.

## Rules

- Look columns up by name, never by position.
- Never rename or remove a column without changing `kTelemetrySchema`.
- New columns go at the end. Readers that look columns up by name keep working, so this needs no schema change.
- Keep the column names, the units and the row function in the same order. `test_rigid_body_telemetry` checks this.
- Files written before the force and torque columns existed have 15 columns. `tools/topdown.py` needs the force columns;
  `tools/plot_run.py` works with either.