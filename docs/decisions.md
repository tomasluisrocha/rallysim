# Modelling decisions

This is the engineering record for the simulator's important modelling
choices. It explains what the model currently means, why the main
simplifications were chosen, and where a future change would affect the
results. It is deliberately more detailed than the project README, but it is
not a diary: implementation details belong in the code and measured output
belongs in the tests or telemetry.

The sources of truth are:

- the headers and scenarios in the implementation;
- the independent checks in `tests/`;
- the file format documented in [telemetry_schema.md](telemetry_schema.md);
- the tool behaviour documented in [tools.md](tools.md).

## Project priorities

The simulator prioritises:

1. **Understandability** - each model should be small enough to inspect.
2. **Determinism** - equal inputs should produce repeatable results.
3. **Testability** - simplified models are useful when their limits are known.
4. **Extensibility** - the bicycle and single-track models provide references
   for the more detailed four-wheel model.

This is why the project does not begin with a large production vehicle stack.
The model is built in layers so that a new effect can be compared with a
simpler model and with a physical or mathematical expectation.

## Foundational conventions

| Area | Decision | Reason |
|---|---|---|
| Language | C++17 for the header-only core; Python for analysis and plots | Keeps the simulation portable and makes the analysis tools easy to iterate |
| Time | Fixed physics step, 1 ms by default and set per scenario (`run.dt`); wheel spin uses smaller internal substeps | A fixed clock makes scripted runs reproducible, while wheel spin needs finer integration |
| Units | SI units throughout the core and telemetry | Avoids hidden conversions between model components |
| Coordinates | ISO 8855: x forward, y left, z up | Gives the vehicle quantities a consistent automotive convention |
| Orientation | Body-to-world Hamilton quaternions, w first | Avoids Euler-angle singularities in the rigid-body state |
| State frames | Position and velocity in the world frame; angular velocity and torque in the body frame | The inertia is constant in the body frame, where Euler's equations are simplest |
| Wheel order | FL, FR, RL, RR everywhere | Makes telemetry, load transfer and tests unambiguous |
| Floating point | No fast-math; precise floating point; finite values are required | Numerical failures should be visible rather than converted into plausible output |
| Rendering | No renderer in the simulator; Python tools replay telemetry | Keeps visualisation separate from physics and makes generated views reproducible |
| Output | CSV runs and generated images live under `out/` | The simulator remains a command-line program and generated data stays disposable |

## Sign conventions

- Steering angle is positive when the wheels turn left.
- Yaw angle and yaw rate are positive counter-clockwise when viewed from above.
- Lateral velocity, sideslip, lateral force and lateral acceleration are
  positive to the vehicle's left.
- Slip angle is wheel heading minus the direction of contact-patch travel.
  A positive slip angle therefore produces a positive lateral force.
- Longitudinal force is positive forwards. Braking opposes wheel rotation and
  never reverses it.
- Slip ratio is positive when the wheel surface is faster than the road
  (driving); a locked wheel moving forwards approaches -1.
- Suspension deflection is compression.
- Positive roll means the right side is down; positive pitch means the nose is
  down.
- Torque and angular velocity use the body frame and the right-hand rule.

## Model scope

### Vehicle models

The project keeps three levels of vehicle model:

- **Bicycle:** a kinematic reference for path geometry and low-load
  comparisons. It is useful below the limit, where tyre saturation and load
  transfer are not the main question.
- **Single-track:** a dynamic two-axle model with body velocity, slip angles,
  lateral tyre forces and quasi-static load transfer. It is a compact
  reference for the four-wheel car.
- **Four-wheel:** individual wheels, wheel rotation, combined-slip tyres,
  drive and brake torques and load transfer, with a choice of three vertical
  models (below). This is the main exploratory model.

The four-wheel model is not yet a complete rally simulator. The ground is
currently flat, terrain and surface data are reserved for later work, and the
engine, gearbox and differential are represented by simple controllable
torques.

### Tyres

The tyre supports three curve families: linear, peak-slide and TMeasy-inspired.
The car currently uses the TMeasy-inspired curve, while the other curves
remain useful for comparisons and tests.

Longitudinal and lateral slip share one grip: the curve is read once at the
combined slip and the force is split between the two directions, so it never
exceeds the grip (the linear curve instead scales an oversized force back onto
the friction circle). The tyre also models relaxation length, so force builds
over distance rather than appearing instantaneously. These choices are intentionally simple enough to
compare against the tyre rig and to replace independently later.

### Loads and suspension

The four-wheel car finds its vertical loads in one of three ways, chosen with
`vertical` in a scenario file:

- **`planar`**: gravity is cancelled and the body stays flat; the loads come
  from a quasi-static estimate of longitudinal and lateral load transfer. It
  compares directly with the single-track model.
- **`springs`** (the default): gravity acts and each corner is a massless
  vertical spring and damper, so heave, pitch and roll emerge from the
  rigid-body motion. A corner can push the body up but never pulls it down.
- **`unsprung`**: each corner is a quarter-car. A wheel mass sits on a tyre
  spring under a vertical strut: a spring, a damper with separate bump and
  rebound rates (`rebound_ratio_*` times `damper_*`), a progressive bump stop
  and a droop stop that limits how far the wheel can hang. The tyre only
  pushes; the strut may also pull, through its damper and the droop stop. The
  bump stop only pushes. An anti-roll bar per axle (`antiroll_*`, 0 by
  default) puts equal and opposite forces on the axle's two wheels in
  proportion to the difference of their strut travel. It resists roll and
  moves lateral load transfer towards its axle, without changing heave or
  pitch.

All three are kept, so each can be compared with the one before it.

### Wheel masses

In the `unsprung` model the whole mass moves the car along the ground, while
vertically the body has the sprung mass (the car less its four wheels) and each
wheel its own. The wheels' horizontal inertia about the body's centre of mass
is left out: about 0.4% of the pitch inertia and 2% of the roll inertia. A
coupled mass matrix (the wheels as point masses solved together with the body)
would remove only that, at the cost of replacing the rigid-body step and losing
the closed forms the model is tested against.

The defaults keep the damper symmetric (rebound ratio 1), and the bump-stop
values are placeholders that no shipped scenario reaches, not measured values
for any car. The loader refuses an unsprung car whose corner step number
reaches 2 at the run's step: a screening rule for the step's stability (a
light wheel on a stiff tyre needs a small step), not a bound and not a measure
of accuracy.

What the tests establish, within the model's own assumptions:

- at rest the car is level and each tyre carries its share of the weight;
- with the centre of mass midway between the axles, pure heave matches the
  standalone two-mass quarter-car within 1e-9 m, and the quarter-car's modes
  are within 1% of their closed forms;
- steady roll is within 3% of the closed form with the spring and tyre in
  series;
- with anti-roll bars, the steady roll is within 1.5% of the closed form and
  the front share of the lateral load transfer within 0.002 of the share set
  by the axles' roll stiffnesses (spring, bar and tyre), and the bars change
  nothing in heave or pitch;
- a light wheel on a stiff tyre (2 kg on 5 MN/m) reproduces the `springs`
  model to within millimetres, run at a 0.1 ms step (such a stiff corner
  needs a small step);
- drops from 0.3 m and 1 m never gain energy and settle back to the static
  loads;
- with a bump stop, drops in pure heave match an independent fine-step
  quarter-car within 1 mm of travel and 3% of peak stop force, and the energy
  books balance during the landing, while the stop holds energy, not only at
  the end;
- the step screening: linear two-mass cases at a screening number of 3.9 do
  not grow, and the light-wheel car at a step just inside the simple limit
  stays within 20% of its static tyre load.

These are agreements with closed forms and simpler models, not a claim of
accuracy against a real car.

## Architecture and dependency direction

| Folder | Holds |
|---|---|
| `core/` | Maths, rigid-body integration, fixed-step timing, telemetry writer, `.ini` parser, finite-value checks |
| `vehicle/` | The bicycle, single-track and four-wheel models; tyre, wheel, corner, drive and brakes, energy books, telemetry columns, parameter registry |
| `driver/` | Input scripts and closed-loop drivers |
| `runner/` | How a scenario runs: the run loops and the scenario-file loader |
| `runner/builtin/` | The earlier models' scenarios and the tyre rig, written in C++ |
| `scenarios/` | The four-wheel scenario files (`.ini`) that `rallysim <name>` runs |
| `apps/` | The `rallysim` command-line program |
| `tests/` | Focused tests and independent reference checks, one program per file |
| `tools/` | Python tools that read telemetry and never change the physics |

One four-wheel run: the scenario file is loaded and the starting state
checked. Each step then takes the driver's inputs and, when telemetry is on,
writes a row for the state at the start of the step. From that state it
evaluates the drive and brake torques, then each wheel's contact, load, patch
velocity and tyre force; it steps the wheels' spin in sub-steps, applies the
resulting force and torque to the body and, in the `unsprung` model, then moves
each wheel vertically. After the step the run stops with an error if the state
is not finite or the car is where the model cannot represent it (on its side
or roof, or with its centre of mass at or below the ground).

Dependencies point from the application toward the core. `world/` and `data/`
are reserved for future terrain, surfaces and vehicle-data loading.

## Validation policy

Every new effect should have at least one check that does not simply repeat
the implementation:

- closed-form results for geometry, energy, suspension or limiting cases;
- an independent bicycle or single-track comparison where appropriate;
- a finite-state check after every simulation step, and invariant checks
  (such as the energy balance) over whole runs;
- timestep-convergence checks for integration-sensitive behaviour;
- golden runs only after the result has passed an independent plausibility
  check.

Telemetry is part of the debugging surface. New values should be named,
unit-labelled and documented in [telemetry_schema.md](telemetry_schema.md).
The Python tools should read columns by name rather than by position.
