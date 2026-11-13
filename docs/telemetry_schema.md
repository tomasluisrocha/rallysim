# Telemetry schema

Telemetry is a CSV file written by `telemetry_writer` (`core/telemetry.hpp`), one row per physics step
plus one for the final state. The simulator writes it; the tools in `tools/` only read it.

## File layout

```
# schema=6 dt=0.001 units: step[count] time[s] pos.x[m] ...
step,time,pos.x,...
0,0,0,...
```

- Line 1 is a comment: the schema number, the time step `dt` in seconds, optional `key=value` facts about the run
  (see below), and the unit of every column.
- Line 2 is the header: dotted column names, comma separated.
- Every following line is one row of numbers.
- Numbers are written with 17 significant digits, so a value reads back as exactly the same double.
- Every number is finite: the writer refuses a NaN or infinity (an exception naming the column and the row) rather
  than write it, and the runners stop a run whose state stops being finite (`core/finite.hpp`). A file that ends
  early was cut short by that, or because the springs car left what the model can represent (on its side or roof, or
  its CG at or below the ground: no body contact yet); `rallysim` prints why.
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
| every `rallysim params` name | its unit | four-wheel | Every parameter of the car, under the name `--set` and scenario files use (`vehicle/four_wheel_registry.hpp`): geometry, inertia, tyres, suspension, drive and brakes. So a four-wheel file records the whole car that made it, settings included |
| `gravity` | m/s^2 | four-wheel | The gravity the run used |
| `rest_deflection_front`, `rest_deflection_rear` | m | four-wheel on springs or unsprung | The `deflection` of that axle's corners with the car at rest, so travel is `deflection` minus it; in the unsprung model the bump stop engages `bump_travel_*` past it and the droop stop `droop_*` before it |
| `vertical` | - | four-wheel | The vertical model: 0 planar, 1 springs, 2 unsprung (`vertical_model` in `vehicle/four_wheel.hpp`) |
| `springs` | - | four-wheel | 1 if the car is on springs (the springs and unsprung models), 0 for the flat (planar) model |
| `curve_front`, `curve_rear` | - | four-wheel | Each tyre's curve: 0 linear, 1 peak_slide, 2 tmeasy (`tyre_curve` in `vehicle/tyre.hpp`) |
| `tmeasy_<axis>_<field><n>_<axle>` | | four-wheel | The tyres' TMeasy tables: `<axis>` x (along) or y (across); `<field>` `stiffness` (N per unit slip ratio, N/rad), `peak_slip`, `sliding_slip` (slip ratio along, tangent of the slip angle across), `sliding_ratio` (-); `<n>` 1 at the nominal load, 2 at twice it; `<axle>` front or rear (32 keys) |

Files written before these keys existed have none; `tools/topdown.py` then draws the default car, and
`tools/tyre_points.py` uses the default tyre.

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

Schema 6 inserted `tyreDefl`, `wheelZ` and `stopF` after `damperF` in each wheel's block, and line 1 of the four-wheel
files gained `vertical`. The four `arbF` columns were later appended after `energy.residual`, so the schema number
did not change.

Schema 5 inserted `net.Fx`, `net.Fy` and `yaw_moment` after `a_lat`, and `grip` after `Fy` in each wheel's block,
and line 1 of the four-wheel files lists every parameter.

Schema 4 regrouped the four-wheel body columns (see the four-wheel table) and added `w`, `vel.z`, `roll_rate`,
`pitch_rate` and `a_long`.

Schema 3 inserted columns into the four-wheel files rather than appending them: `pos.z`, `roll` and `pitch` after
`a_lat`, and `deflection`, `deflVel`, `springF` and `damperF` at the end of each wheel's block. Readers that look columns
up by name (every tool in `tools/`) read schema 2 to 6 files alike; anything that counts columns does not.

Schema 2 renamed the single-track columns `alpha_f`, `alpha_r`, `fy_f`, `fy_r` and `fz_fl` to `fz_rr` to the dotted
names above, so a plot script can build a name from an axle or wheel label.

The columns from `steer` on belong to the step that starts from that row's state. The slip angles use a
forward speed of at least `kMinForwardSpeed` (1 m/s), so below that `u` and the slip angles do not
match; the model is not meant for that range. `yaw`, `steer`, `speed` and `beta` are the columns
`tools/topdown.py` needs, so it also plays these files. `beta` and `yaw_rate` have the same names and
meaning as in the bicycle files, which makes the two models easy to overlay.

## Columns (four-wheel car)

The file layout is the same. The body columns come in groups: time, the driver's inputs, where
the car is, how it moves, then values worked out from those. Each wheel then adds a block of columns, and the energy
books end the row. Columns named as in the single-track files mean the same thing, so the two models can be overlaid;
the pedals replace `drive_force`.

| # | Name | Unit | Meaning |
|---|---|---|---|
| 1 | `step` | count | Step index, starting at 0 |
| 2 | `time` | s | `step * dt` |
| 3 | `steer` | rad | Front wheel angle the model used (limited to max_steer), positive = left; both front wheels |
| 4 | `throttle` | 0-1 | Throttle pedal, as given (the speed controller's output when a scenario holds speed) |
| 5 | `brake` | 0-1 | Brake pedal, as given |
| 6 | `handbrake` | 0-1 | Handbrake, as given |
| 7 | `pos.x` | m | Centre of gravity, world frame |
| 8 | `pos.y` | m | |
| 9 | `pos.z` | m | Height of the CG above the ground (0 in the planar model) |
| 10 | `roll` | rad | Body roll, positive with the right side down (leaning out of a left turn) |
| 11 | `pitch` | rad | Body pitch, positive nose down (diving under braking) |
| 12 | `yaw` | rad | Heading of the body, counter-clockwise from +x |
| 13 | `u` | m/s | Forward speed of the CG, along the body's x axis |
| 14 | `v` | m/s | Sideways speed of the CG, along the body's y axis, positive to the left |
| 15 | `w` | m/s | Speed of the CG along the body's z axis, positive up. Tilts with the body, so not the climb rate: see `vel.z` |
| 16 | `roll_rate` | rad/s | Angular velocity about the body's x axis (ISO p), positive with the right side going down |
| 17 | `pitch_rate` | rad/s | Angular velocity about the body's y axis (ISO q), positive with the nose going down |
| 18 | `yaw_rate` | rad/s | Angular velocity about the body's z axis (ISO r), positive turning left |
| 19 | `speed` | m/s | `hypot(u, v)`: the speed of the CG, leaving out `w` |
| 20 | `beta` | rad | Body sideslip: angle between the heading and the velocity of the CG |
| 21 | `vel.z` | m/s | Vertical speed of the CG, world frame, positive up: the heave rate |
| 22 | `a_long` | m/s^2 | The force on the body along its heading on the ground, over the mass (the tyres' net push); positive forwards |
| 23 | `a_lat` | m/s^2 | `speed * yaw_rate`; positive when turning left |
| 24 | `net.Fx` | N | The four tyre forces summed, along the body's x axis |
| 25 | `net.Fy` | N | Along the body's y axis, positive to the left |
| 26 | `yaw_moment` | N m | Their moment about the CG, about the body's z axis: what turns the car; positive turning left |
| 27-42 | `wheel.FL.*` | | Front-left wheel, the sixteen fields below |
| 43-58 | `wheel.FR.*` | | Front-right wheel |
| 59-74 | `wheel.RL.*` | | Rear-left wheel |
| 75-90 | `wheel.RR.*` | | Rear-right wheel |
| 91-101 | `energy.*`, `work.*` | J | The energy books, below |
| 102 | `wheel.FL.arbF` | N | Anti-roll bar force at the front-left corner: positive pushes the body up; the two wheels of an axle carry opposite values; 0 except in the unsprung model with a bar |
| 103 | `wheel.FR.arbF` | N | Front right |
| 104 | `wheel.RL.arbF` | N | Rear left |
| 105 | `wheel.RR.arbF` | N | Rear right |

Columns 13-18 are the six ISO vehicle velocities (`body_velocity`), in the body's axes, so they tilt with the body;
the position, the angles and `vel.z` are in the world frame. So a rate is its angle's slope only while the car is
level: in a steady turn, rolled, `pitch_rate` is `yaw_rate * tan(roll)` although the pitch does not change. `a_long` comes from the forces and `a_lat` from the
motion (`speed * yaw_rate`, exact in a steady turn), so in fast transients they are not worked out the same way.

Each wheel block, in this order:

| Field | Unit | Meaning |
|---|---|---|
| `omega` | rad/s | Wheel spin, positive rolling forward |
| `slipRatio` | - | Steady (kinematic) slip ratio of the contact patch, `(omega R - vx) / max(|vx|, |omega R|)`: positive driving, -1 a locked wheel sliding forward. Meaningless at a standstill |
| `slipAngle` | rad | Steady slip angle, `atan(-vy / max(|vx|, |omega R|))`: positive when the wheel points left of its path |
| `Fz` | N | Vertical load on the wheel. Planar: the quasi-static load-transfer force. Springs: the corner force (spring + damper). Unsprung: the tyre spring's load |
| `Fx` | N | Tyre force along the wheel |
| `Fy` | N | Tyre force across the wheel, positive to its left |
| `grip` | N | The most force the tyre can take at this load, `mu (1 - k (Fz - Fz0) / Fz0) Fz`: `hypot(Fx, Fy) / grip` is the grip in use |
| `driveTorque` | N m | Drive torque the pedals put on this wheel |
| `brakeTorque` | N m | Brake torque on this wheel (a magnitude; it opposes the spin) |
| `deflection` | m | How far the corner's spring is compressed; zero or less means the wheel is off the ground. In the unsprung model it is the strut's compression, not the tyre's: a wheel off the ground can still have a positive `deflection` (see `tyreDefl`) |
| `deflVel` | m/s | How fast it is being compressed |
| `springF` | N | Spring force |
| `damperF` | N | Damper force, positive while compressing; in the springs model `Fz` is spring + damper, never below zero; in the unsprung model `Fz` is the tyre's load, from the tyre spring, not the strut's |
| `tyreDefl` | m | The tyre's compression; zero or less means it is off the ground. 0 except in the unsprung model |
| `wheelZ` | m | Height of the wheel centre above the ground. 0 except in the unsprung model |
| `stopF` | N | The stops' force: negative from the droop stop (it pulls the wheel up), positive from the bump stop (it pushes the body up); never both. 0 except in the unsprung model |

`Fx` and `Fy` are in the wheel's own frame, so on a steered front wheel they are turned by `steer` from the body
axes. The slips are the patch's steady slips, worked out from the wheel's spin and the patch velocity. The wheel's own
relaxed slips (the twist of the tyre, which the force comes from) are not logged: they are capped where the force stops
changing (`slip_cap_scale` in `vehicle/tyre.hpp`: at full sliding while rolling, at the peak at rest), so a locked
tmeasy wheel's relaxed slip is about -0.8 while it slides and the slip at its peak once stopped, not -1. As in the
other files, a row's inputs and forces belong to the step that starts from that row's state.

The energy books (`vehicle/four_wheel_energy.hpp`), all in J. The stored terms are that row's state; the
work columns add up every step before it, from the start of the run:

| Column | Meaning |
|---|---|
| `energy.translation` | Body: ½ m \|v\|². Unsprung model: ½ m (vx² + vy²) + ½ m_s vz² + each wheel's ½ m_u vz² |
| `energy.rotation` | Body: ½ (Ix ωx² + Iy ωy² + Iz ωz²), body frame |
| `energy.gravity` | Body: m g z (springs model; 0 in the planar model, which cancels gravity). Unsprung model: m_s g (height of the sprung body's CG) + each wheel's m_u g z |
| `energy.springs` | Corner springs: ½ k x² for each compressed one (springs model). Unsprung model: the strut spring, the tyre spring (while on the ground), the droop stop and the bump stop (each while engaged), and each axle's anti-roll bar, ½ kb (x_L − x_R)² |
| `energy.wheels` | Wheel spin: ½ I ω² for each wheel |
| `energy.total` | The sum of the five above |
| `work.drive` | Put in by the drive: drive torque × spin |
| `work.brake` | Taken out by the brakes: brake torque × \|spin\| (0 while a wheel is locked) |
| `work.damper` | Taken out by the dampers: the corner force beyond its spring × compression speed. Unsprung model: the strut damper, plus the tyre's damping while it is on the ground |
| `work.tyre` | Taken out in the contact patches: tyre force × slip velocity |
| `energy.residual` | `energy.total` − its value in the first row − (drive − brake − damper − tyre): near zero |

The residual is a first-order integration error, not exactly zero, and halves when dt halves; a residual that does not
shrink with dt is a physics bug. `tests/test_energy.cpp` bounds it on every four-wheel scenario.

## Tyre rig file

`rallysim tyre_rig` (`runner/builtin/tyre_rig.hpp`) writes `out/rig/tyre_rig.csv`: the force curves of one tyre, swept over
slip and load with no car and no time. It is not a telemetry file and has no schema number.

```
# tyre_rig
curve,sweep,fz,slip_ratio,slip_angle_deg,fx,fy
linear,x,1500,-1.0000,0.00,-1575,0
```

- Line 1 is `# tyre_rig`; line 2 the header; then one row per point.

| Column | Unit | Meaning |
|---|---|---|
| `curve` | text | `linear`, `brush` (peak_slide with `peak_slip` 3, `sliding_ratio` 1), `peak_slide`, `tmeasy`: each with the `tyre_params` defaults (`tyre_rig_curves()`) |
| `sweep` | text | `x`: slip ratio -1 to 1 in steps of 0.002, no slip angle; `y`: slip angle -30 to 30 deg in steps of 0.1, no slip ratio; `combined`: slip ratio -1 to 1 at slip angles of 2, 4, 6 and 8 deg |
| `fz` | N | Vertical load: 1500, 3000, 4500 and 6000 for `x` and `y`; the nominal load (3000) for `combined` |
| `slip_ratio` | - | Slip ratio given to `tyre_forces` |
| `slip_angle_deg` | deg | Slip angle; `tyre_forces` is given its tangent, the lateral slip |
| `fx`, `fy` | N | The tyre force in the wheel frame, along and to the left (6 significant digits) |

`tools/tyre_plot.py` draws it; `tools/simdata.py` checks its own copy of the tyre against every row.

## Rules

- Look columns up by name, never by position.
- Never write a NaN or infinity; `telemetry_writer::row` enforces it.
- Never rename or remove a column without changing `kTelemetrySchema`.
- New columns go at the end. Readers that look columns up by name keep working, so this needs no schema change.
- Keep the column names, the units and the row function in the same order. `test_rigid_body_telemetry` checks this.
- Files written before the force and torque columns existed have 15 columns. `tools/topdown.py` needs the force columns;
  `tools/plot_run.py` works with either.