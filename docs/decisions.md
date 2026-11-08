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
| Time | Fixed 1 ms physics step; wheel spin uses smaller internal substeps | A fixed clock makes scripted runs reproducible, while wheel spin needs finer integration |
| Units | SI units throughout the core and telemetry | Avoids hidden conversions between model components |
| Coordinates | ISO 8855: x forward, y left, z up | Gives the vehicle quantities a consistent automotive convention |
| Orientation | Body-to-world Hamilton quaternions, w first | Avoids Euler-angle singularities in the rigid-body state |
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
  drive and brake torques, load transfer and corner springs (by default; a
  planar mode without them is kept for comparison). This is the main
  exploratory model.

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

The planar four-wheel mode uses a quasi-static estimate of longitudinal and
lateral load transfer. The spring mode applies gravity and a spring/damper at
each corner, allowing heave, pitch and roll to emerge from the rigid-body
motion. Both modes are retained: the planar mode is easier to compare directly
with the single-track model, while the spring mode is more representative of
the vehicle's vertical behaviour.

## Architecture and dependency direction

- `core/` contains maths, rigid-body integration, fixed-step timing,
  telemetry and finite-value checks.
- `vehicle/` contains bicycle, single-track and four-wheel models plus tyres,
  wheels, brakes and suspension.
- `driver/` contains scripted inputs and controllers.
- `scenarios/` combines a model, driver and run length.
- `apps/` contains the `rallysim` command-line program.
- `tests/` contains focused tests and independent reference checks.
- `tools/` reads telemetry and never changes the physics.

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
