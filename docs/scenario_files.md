# Scenario files

A scenario file describes one four-wheel run as text: the car, where it starts, and what the driver does. It is
read when the run starts, so a run can be written and tuned without rebuilding.

```
rallysim fw_brake                         scenarios/fw_brake.ini
rallysim my_run.ini [out.csv]             any file, by path
rallysim fw_brake --set spring_front=50kN/m --set run.dt=0.5ms
rallysim params fw_hairpin                the car a file sets up
rallysim list                             every scenario, built-in and scenarios/*.ini
```

The telemetry goes to `out/runs/<file name>.csv` unless a path is given. `scenarios/fw_showcase.ini` uses nearly
every feature below.

## Lines

| Line | Meaning |
|---|---|
| `[name]` | starts a section: lower case letters, digits and `_` |
| `key = value` | an entry of the section above it; spaces around the key and the value do not matter |
| blank, or only a comment | ignored |

- `#` starts a comment anywhere on a line, so a value cannot hold a `#`.
- A value ending in `,` (once its comment is removed) continues on the next line, so a long timeline can be split
  with a comment on each point.
- Keys are lower case letters, digits and `_`, like section names.
- Every entry needs a section, and a key or a section may appear only once.
- UTF-8 (with or without a byte-order mark), `\n` or `\r\n` line ends.

## Values

Each key reads its value as one type:

| Type | Written as | Examples |
|---|---|---|
| number | a decimal number, optionally with an exponent, optionally followed by a unit | `1200`, `-0.4`, `6e4`, `.5`, `50 km/h`, `25deg` |
| vector | three numbers in brackets; a unit after the bracket applies to all three, or each number has its own | `(10, -2, 0.77) m`, `(50 km/h, 0, 0)` |
| word | one of a fixed list | `tmeasy`, `planar` |
| text | everything after `=`, up to a comment | `Crest jump into a hairpin` |
| timeline | `value @ time` points, separated by commas | `0 @ 0, 0.4 @ 1.2 s, 0 @ 2500 ms` |

A number without a unit is in the key's own unit, given in the tables below. A unit is converted to it, and must
measure the same thing: `spring_front = 50 km/h` is an error. Hex, `inf` and `nan` are not numbers, and the whole
value must be read: `6e4abc` is an error, not 60000.

### Units

| Measures | Key's unit | Also accepted |
|---|---|---|
| angle | `rad` | `deg` |
| time | `s` | `ms` |
| length | `m` | `mm`, `cm`, `km` |
| speed | `m/s` | `km/h` |
| angular speed | `rad/s` | `deg/s`, `rpm` |
| mass | `kg` | `t` |
| force | `N` | `kN` |
| torque | `N m` | `kN m` |
| power | `W` | `kW`, `hp` |
| spring rate | `N/m` | `kN/m`, `N/mm` |
| damper rate | `N s/m` | `kN s/m` |
| cornering stiffness | `N/rad` | `kN/rad` |
| inertia | `kg m^2` | |
| rate | `1/s` | |
| ratio | none | `%` |

A unit may follow the number with or without a space (`25deg`, `25 deg`). Units with a space inside take exactly
one (`kN m`, not `kN  m`).

### Timelines

- Times start at 0 or later and increase strictly.
- The value is linear between points, the first value before the first point, the last value after the last.
- A sudden change is two close points: `0 @ 1.3 s, 1 @ 1.35 s`.
- A single number is a constant: `throttle = 0.2`.
- Each point's value takes the key's unit and its time takes `s`: `30 km/h @ 2500 ms`.

## Sections

Unknown sections and keys are errors. A key that is not given takes its default.

### `[scenario]`

| Key | Type | Meaning |
|---|---|---|
| `name` | text | for people |
| `description` | text | for people |

### `[run]`

| Key | Type | Unit | Default | Meaning |
|---|---|---|---|---|
| `model` | word | | `four_wheel` | the only model read from files |
| `duration` | number | s | required | length of the run; a whole number of steps |
| `dt` | number | s | `0.001` | the fixed step, above 0 |

### `[start]`

| Key | Type | Unit | Default | Meaning |
|---|---|---|---|---|
| `position` | vector | m | `(0, 0, cg_height)` | the CG, world frame; a z above `cg_height` starts the car in the air (z is 0 in the planar model) |
| `heading` | number | rad | `0` | counter-clockwise from +x |
| `pitch` | number | rad | `0` | positive nose down |
| `roll` | number | rad | `0` | positive right side down |
| `speed` | number | m/s | `0` | along the heading; not together with `velocity` |
| `velocity` | vector | m/s | `(0, 0, 0)` | the CG, world frame |
| `angular_velocity` | vector | rad/s | `(0, 0, 0)` | body frame: roll, pitch and yaw rates |

The wheels start rolling at the body's forward speed, so no tyre starts out slipping. The default height follows
the file's `cg_height`.

### `[car]`

Every number `rallysim params` lists, in the unit it shows: `mass`, `lf`, `cg_height`, `mu_front`, `fz0_rear`,
`relaxation_y_front`, `spring_rear`, `drive_split_front` and the rest. Changing the mass, the axle positions, a tyre
stiffness or a nominal load refits the tyres to the new loads.

A number the car does not use is an error, because it would change nothing: the springs and dampers only with
`vertical = springs` or `unsprung`, the wheel's `unsprung_mass_*`, `tyre_rate_*`, `tyre_damping_*`, `droop_*`, `rebound_ratio_*`, `bump_travel_*`, `bump_stop_length_*`, `bump_stop_rate_*` and `droop_stop_rate_*` only
with `vertical = unsprung` (whose four wheel masses must add up to less than `mass`), `front_transfer_share` only with `vertical = planar`, and a tyre's `peak_slip`, `sliding_slip`
and `sliding_ratio` only when that axle is on `peak_slide`. This is judged after the words below are read, wherever
they are in the file; `rallysim params <scenario>` marks what a car does not use.

An unsprung car is refused when a corner's step number reaches 2 at the run's `dt` (a light wheel, a stiff tyre or
stop, or strong damping for the step); the message gives the largest `dt`. The number screens step stability only, not
accuracy.

`rebound_ratio_*` multiplies `damper_*` while the strut extends (1: the same in both directions). `bump_travel_*` is the
compression past rest where the bump stop starts, and its stiffness rises to `bump_stop_rate_*` over
`bump_stop_length_*`; `droop_stop_rate_*` is the droop stop's stiffness. The bump-stop defaults are placeholders that no
shipped scenario reaches, not measured values for the car.

A number outside its range is an error too; nothing is clamped:

| Range | Keys |
|---|---|
| above 0 | `mass`, `inertia_x/y/z`, `lf`, `lr`, `track_front/rear`, `cg_height`, `max_steer`, `wheel_radius`, `wheel_inertia`, `fz0_*`, `cx_*`, `calpha_*`, `relaxation_*`, `spring_*`, `unsprung_mass_*`, `tyre_rate_*`, `droop_*`, `rebound_ratio_*`, `bump_travel_*`, `bump_stop_length_*`, `bump_stop_rate_*`, `droop_stop_rate_*`, `sliding_slip_*`, `max_drive_power` |
| 0 or more | `mu_*`, `k_*`, `damper_*`, `tyre_damping_*`, `max_drive_torque`, `max_brake_torque`, `max_handbrake_torque` |
| 0 to 1 | `front_transfer_share`, `drive_split_front`, `brake_bias_front`, `sliding_ratio_*` |
| 1.5 to 3 | `peak_slip_*` |

On a `peak_slide` axle, `sliding_slip` must also be above `peak_slip`.

The words:

| Key | Values | Default |
|---|---|---|
| `tyre_curve` | `linear`, `peak_slide`, `tmeasy`, for both axles | `tmeasy` |
| `tyre_curve_front`, `tyre_curve_rear` | the same, one axle; not together with `tyre_curve` | |
| `vertical` | `springs`, `planar`, `unsprung` | `springs` |

### `[driver]`

| Key | Type | Unit | Default | Meaning |
|---|---|---|---|---|
| `steer` | timeline | rad | `0` | front wheel angle, positive left; limited to `max_steer` |
| `throttle`, `brake`, `handbrake` | timeline | - | `0` | pedals, 0 to 1 (or `%`) |
| `hold_speed` | timeline | m/s | off | target speed for a speed controller, which then works the throttle and brake (so not together with them), and lifts off while the handbrake is pulled |
| `speed_gain` | number | 1/s | `10` | how hard the speed controller pushes: drive force per kg per m/s below the target |
| `hold_heading` | number | rad | off | target heading for a steering driver that countersteers to catch a slide; needs `hold_heading_from` |
| `hold_heading_from` | number | s | | when that driver takes over the steering |
| `heading_gain` | number | 1/s | `2.5` | yaw rate asked for per radian of heading still to go |
| `max_yaw_rate` | number | rad/s | `1.2` | the most yaw rate it asks for |
| `yaw_rate_gain` | number | s | `0.35` | steer per yaw rate error |

Both controllers are simple. The speed controller knows nothing about grip: in a slide it floors the throttle
while the car is below its target, which can spin a rear-driven car (see `fw_showcase.ini`).

## `--set`

`--set name=value` changes a `[car]` key; `--set section.key=value` changes any other one. It replaces the file's
value, or adds it, and uses the same value syntax, units and timelines included (quote a value with spaces:
`--set "driver.steer=0 @ 0, 0.3 @ 2 s"`).

## Errors

A file with any error does not run and writes no telemetry. Every error is listed at once, in line order, as
`file:line: key: what`; a `--set` shows as `--set`:

```
--set: mass: not a number: 'abc'
my_run.ini:2: duration: 1.0003 s is not a whole number of 0.001 s steps
my_run.ini:4: speed: give speed or velocity, not both
my_run.ini:7: mass: must be above 0
my_run.ini:8: spring_front: unit km/h is not N/m
my_run.ini:9: unknown section [track]
```

## Example

```ini
# A steady circle of about 24 m radius at 30 km/h, on gravel.
[scenario]
name = gravel_circle

[run]
duration = 20 s

[start]
speed = 30 km/h

[car]
mu_front = 0.75
mu_rear = 0.75

[driver]
hold_speed = 30 km/h
steer = 0 @ 1 s, 6 deg @ 1.5 s
```
