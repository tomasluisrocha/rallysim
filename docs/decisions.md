# Decisions and conventions

The book asks for the first decisions to be made before any code (chapter 1) and for every sign to be
written down before it is coded (section 5.1). This file holds both. Record any change in the log at the end.

## First decisions (chapter 1)

| Decision | Choice here | Where it shows |
|---|---|---|
| Language | C++17 for the core (header-only library `rallysim_core`), Python for plots | `CMakeLists.txt`, `tools/` |
| Physics timestep | Fixed 1 ms (1000 Hz); the core never takes a variable dt | `dt = 0.001` in every scenario, `core/fixed_step.hpp` |
| Units | SI everywhere in the core; conversions only at the edges | all headers, `docs/telemetry_schema.md` |
| Coordinates | ISO 8855: x forward, y left, z up | `core/vec3.hpp`, `core/wrench.hpp` |
| Ground truth | Closed forms and independent models in the tests; NGP and RBR only as plausibility checks | `tests/` |
| Rendering | None yet; a 2D top-down replay of the telemetry | `tools/topdown.py` |
| Determinism | Same inputs give identical output on one machine; `/fp:precise` and `-ffp-contract=off`, no fast-math | `CMakeLists.txt`, golden tests |
| Licensing | Own code and own parameters only | |

## Sign conventions (section 5.1)

| Quantity | Positive means | Note |
|---|---|---|
| Steering angle | Wheel turns left (counter-clockwise from above) | Matches positive yaw. Limited to `max_steer` |
| Yaw angle | Counter-clockwise from world +x | `yaw_angle(q)` |
| Yaw rate | Counter-clockwise from above | Right-hand rule about z; `omega.z` in the body frame |
| Lateral velocity `v` | CG moving to the car's left | Body frame |
| Body sideslip `beta` | Velocity of the CG points left of the heading | `atan2(v, u)` |
| Slip angle | Wheel heading minus the direction the axle moves | So a positive slip angle gives a positive (leftward) force, which reduces it |
| Lateral tyre force | To the left of the wheel | Axle frame, sideways to the wheel |
| Lateral acceleration `a_lat` | Turning left | `speed * yaw_rate` |
| Longitudinal force / `drive_force` | Pushes the car forward | Negative brakes. Applied at the rear axle, limited by its grip |
| Load transfer | Accelerating moves load to the rear, a left turn to the right wheels | `axle_loads`, `loads_per_wheel` |
| Quaternion | Body to world, Hamilton product, w first | `core/quat.hpp` |
| Torque and angular velocity | Body frame, right-hand rule | `core/wrench.hpp`, `core/rigid_body.hpp` |
| Wheel spin `omega` | Rolling forward | rad/s about the wheel axle |
| Slip ratio | Wheel surface faster than the road (driving) | `(omega Re - vx) / max(|vx|, |omega Re|)`; -1 is a locked wheel going forward, +1 a wheel spinning at standstill |
| Lateral slip | Wheel points left of where its contact patch moves | Tangent of the slip angle, so the same sign as the slip angle |
| Tyre forces `Fx`, `Fy` | Forward and to the left, in the wheel frame | Positive slip gives positive force |
| Drive torque | Pushes the wheel to roll forward | |
| Brake torque | Always opposes the wheel's spin | Given as a magnitude >= 0; can stop the wheel but never reverse it |
| Wheel order | FL, FR, RL, RR | Fixed everywhere, for determinism |
| Suspension deflection | Compression | Not modelled yet (phase 4, step 6) |

## Code layout

| Folder | Holds | May include |
|---|---|---|
| `core/` | Maths and generic machinery: vec3, quat, wrench, rigid body, fixed-step clock, telemetry writer | `core/` only |
| `vehicle/` | The car models and their telemetry rows: bicycle, single-track | `core/` |
| `driver/` | What produces the inputs: keyframe scripts, speed-hold controller | `core/`, `vehicle/` |
| `scenarios/` | One model + one driver + one script + a run length, and the runners | all of the above |
| `apps/` | The `rallysim` program | `scenarios/` |
| `tests/` | One `test_<name>.cpp` per area, registered with `rallysim_test(<name>)`, plus shared helpers | anything |

Dependencies point one way only. `world/` (terrain, surfaces) and `data/` (car files and their loader) are
created when the first phase needs them. Runs are written to `out/`, which git ignores.

## Decision log

| Date | Decision | Why |
|---|---|---|
| 2026-11 | Phase 2 bicycle integrates the CG, with the heading taken half way through the step | Second-order path accuracy; the circle closes within micrometres. The book's rear-axle form is tested as the `lr = 0` case |
| 2026-11 | Phase 3 single-track runs on the 3D rigid body with gravity cancelled, not on separate planar equations | Reuses the phase 1 integrator; an independent planar model in the tests checks it |
| 2026-11 | Load transfer is quasi-static, from estimated accelerations: `a_x` from the traction-limited drive force, `a_y = u r` | Costs nothing and avoids a force-load loop; ignores the v-dot part of fast transients (book chapter 15 warns about this) |
| 2026-11 | Load sensitivity is linear: `mu(Fz) = mu0 (1 - k (Fz - Fz0) / Fz0)`, `k = 0.1`, `Fz0 = 3000 N` | The simplest form the book asks for in 6.3. Values are placeholders, not fitted |
| 2026-11 | The rear axle shares its grip between drive force and side force (friction circle) | So every force obeys the friction limit (6.3 gate) |
| 2026-11 | Slip angles use a forward speed of at least 1 m/s | Placeholder; the book (chapter 15) recommends blending to the kinematic model below 3 to 5 m/s, to do before the car has to start from rest |
| 2026-11 | One `keyframe_script<T>` template for every scripted input, with a `lerp` per input type | Replaces three copies of the same interpolation; phase 4 adds more input types |
| 2026-11 | Phase 4 tyre: linear in slip ratio and lateral slip, scaled together onto the friction circle when the total exceeds `mu(Fz) Fz` | Book 7.2 rungs 1 to 3; the simplest combined slip that is easy to test. Phase 5 replaces `vehicle/tyre.hpp` |
| 2026-11 | Tyre values per tyre: `Cx = 90 kN`, `C_alpha = 30 kN/rad` (front; 35 rear to match the single-track), `mu = 1`, `k = 0.1`, `Fz0 = 3000 N` | `Cx` from the book (ch. 14); the rest halves of the single-track axle values, so the phase 4 gate compares like with like. Real tarmac tyres are stiffer (50-70 kN/rad), gravel softer |
| 2026-11 | Relaxation lengths `0.2 m` longitudinal, `0.4 m` lateral | Inside the book's 0.2-0.5 m (ch. 14); the longitudinal one is usually the shorter |
| 2026-11 | Relaxed slips, `sigma ds/dt + V s = slip velocity`, `V = max(|vx|, |omega R|)`, backward Euler | Steady state is the book's slip ratio; finite and stable at standstill and at any dt; builds over distance, not time |
| 2026-11 | Wheel: `R = 0.315 m`, `I = 1.2 kg m^2` | `R` is `EffWheelRadius` in the NGP Fabia files and the book's value. `I` estimated: about 20 kg of tyre and rim at a 0.25 m radius of gyration plus hub and disc; the book quotes 2.0 from RBR files |
| 2026-11 | Brake clamp: if the brake can stop the wheel within the step, it stops it exactly and holds it | Book 7.1: brakes stop the wheel but never reverse it |
| 2026-11 | Phase 4 step 3: four-wheel car kept planar (gravity cancelled, tyre forces at CG height), loads from the same quasi-static transfer as the single-track, drive on the rear wheels shared equally, parallel steer | So it can be compared one to one with the single-track (the chapter 15 gate). Springs come in step 6, the drive split in step 4, Ackermann is an extension |
| 2026-11 | `yaw_angle`, `body_velocity` and `velocity_in_body` moved from the single-track to `core/rigid_body.hpp` | Both cars use them |
