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
| Suspension deflection | Compression | `deflection` in the telemetry; positive pushes the body up |
| Roll | Right side down | ISO 8855, about the forward axis: positive leaning out of a left turn |
| Pitch | Nose down | ISO 8855, about the left axis: positive diving under braking |

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
created when the first phase needs them. Runs and pictures are written to `out/`, which git ignores; its folders and
the tools that fill them are in `tools/README.md`.

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
| 2026-11 | Phase 4 step 4: the car is driven by pedals (throttle, brake, handbrake, 0 to 1); `vehicle/drive_brake.hpp` turns them into wheel torques from `max_drive_torque` 6000 N m, `drive_split_front` (0 RWD default, 0.5 AWD, 1 FWD), `max_brake_torque` 6000 N m, `brake_bias_front` 0.70, `max_handbrake_torque` 3000 N m | Pedals are what a player or an AI produces and they survive phase 7, which replaces only the inside (engine, gearbox, differentials). 6000 N m at the wheels is a rally engine in a low gear; the brakes lock all four wheels; RWD default so the four-wheel car still compares with the single-track |
| 2026-11 | Brake bias 0.70, not 0.65 | Measured: at 0.65 the rears lock before the fronts on mu = 1, because braking at 1 g puts 68% of this car's load on the front. Lower it (about 0.6) for gravel |
| 2026-11 | Load transfer estimate from the wheel torques, each axle's force limited by mu times its load, settled with six fixed-point passes | Gives the right traction limit for any drive split, brake bias and handbrake without a formula per case |
| 2026-11 | The speed controller lifts off the throttle while the handbrake is up | On a rear-drive car the drive torque otherwise beats the handbrake and the rears never lock; a driver puts the clutch in |
| 2026-11 | Drive power limit `max_drive_power` 200 kW: available torque `min(T_max, P / omega)`, with omega the average spin of the driven wheels | Measured: with a fixed torque at any wheel speed a spinning wheel sped up for ever (750 m/s surface speed). A placeholder for an engine through an ideal gearbox; phase 7 replaces it |
| 2026-11 | AWD tests and the launch scenario use a 30/70 front/rear split | Measured: 30% front reaches mu g (9.81 m/s^2), about the front's share of the load at 1 g; 50/50 only 6.76 m/s^2, because with open differentials the unloaded fronts spin and take the engine's speed up. Limited-slip differentials (phase 7) are the real fix |
| 2026-11 | The handbrake scenario is a partial (0.7), short (0.3 s) pull with the wheel straightened after it | Measured: a full pull, or keeping the wheel turned in on throttle, spins the car (no countersteer in a script; locked rears have no side grip left) |
| 2026-11 | The car's geometry and tyre grip go on line 1 of each telemetry file as `key=value` | The viewer draws the wheels where they are and computes how much grip each tyre uses, without copying the C++ defaults |
| 2026-11 | `tools/topdown.py` rewritten: one description for every model (body pose, velocity, a list of wheels with force, load and grip), a dashboard (follow view, path map, wheel loads and grip use, inputs, timeline, play and pause), a static view with snapshots, and GIF export | The old viewer knew two file types, drew one front wheel and one force at the CG, and could not play a run |
| 2026-11 | Phase 4 step 5: the relaxed slips (the tyre's twist) are capped at what the grip can hold, `|(Cx sx, C_alpha sy)| <= mu(Fz) Fz` | Measured: a car that stopped on locked wheels kept a twist of sigma (0.2 m) and was thrown backwards, rocking at 1.6 m/s. Capped, it is left with about 7 mm. The sliding force does not change |
| 2026-11 | Low-speed damping of the contact patch, `damping_x` 5000 and `damping_y` 9500 N s/m, fading out linearly by 2.5 m/s of rolling speed | Soaks up the tyre's spring at standstill: a braked car settles within 0.5 s. `damping_y` is about critical for the lateral tyre spring; `damping_x` is held to `c R^2 dt / I < 0.5` (0.41) because it also acts on the wheel's spin and the explicit spin update would blow up (book ch. 14). An implicit spin update would allow more |
| 2026-11 | The four-wheel telemetry logs the steady (kinematic) slips; `topdown.py` flags spin and slide only when something is rolling | The relaxed slips are capped now, so they no longer show how fast a tyre slides; at a standstill the steady slip is 0/0 |
| 2026-11 | Phase 4 step 6: the car sits on a vertical spring and damper at each corner (`vehicle/corner.hpp`), 50 kN/m and 3800 N s/m, gravity acts and the tyre forces act on the ground | About 2.1 Hz and a damping ratio of 0.5, a typical rally ride. The load transfer, roll and pitch now come out of the springs and take time to build. Checked against closed forms: heave 2.054 Hz (formula 2.055), roll 1.286 deg (1.285), pitch 0.514 deg (0.513) |
| 2026-11 | Each spring hangs from the body straight above its contact patch and is preloaded so the car rests level with its CG at `cg_height` | The car starts in balance; no drop and bounce at t = 0 |
| 2026-11 | The flat model stays as `vertical_model::planar`; springs are the default | The phase 4 gate runs on both: the planar one compares one to one with the single-track, the springs one shows the steady state still matches (yaw rate within 0.13%) |
| 2026-11 | The wheel's direction is the body's heading turned by the steer and laid flat on the ground | A rolled or pitched body must not tilt the tyre forces into the ground (flat ground for now; phase 8 brings a contact normal) |
| 2026-11 | Handbrake flick scenario: 0.55 for 0.3 s (was 0.7) | Measured: with springs, braking pitches the body forward and unloads the rear, so at 0.7 the outer rear locks too and the car spins |
| 2026-11 | Phase 4 step 7: golden runs for the five four-wheel scenarios (on springs), each checked first against something that shares none of its code: the understeer gradient, the roll formula and the single-track for the circle and lane change, `v^2 / 2 mu g` for braking (+3.0%), `mu g` then `P / (m_eff v)` for the launch, recovery for the handbrake flick | A wrong result cannot be frozen in as golden. Values to 1e-9, so a compiler's last-bit rounding does not fail them |
| 2026-11 | Timestep convergence checked on the lane change and on braking at 2, 1 and 0.5 ms | Book 11.2: each halving of dt halved the change (lane change end 7.9e-5 then 3.9e-5 m; braking stop 8.6 then 4.1 mm), the first-order behaviour of semi-implicit Euler |
| 2026-11 | Each wheel is sub-stepped: 4 spin updates of 0.25 ms per 1 ms body step (`wheel_params::substeps`), the body getting each wheel's average tyre force over the step | Book 5.4 and 14.4: the wheel (1.2 kg m^2 with ~3 kN at its rim) changes far faster than the body. Measured: with tyre damping of 30 kN s/m one 1 ms step makes the spin chatter (sign flip every step), four sub-steps settle it. Also halved the timestep error at 1 ms (lane change end 7.9e-5 -> 3.5e-5 m, brake stop 8.6 -> 2.2 mm) |
| 2026-11 | `damping_x` raised from 5000 to 18 000 N s/m | Allowed by the sub-steps (`c R^2 (dt / 4) / I` = 0.37); about 0.8 of critical. A locked-wheel stop on the rig now rolls back only the tyre's twist (6.6 mm, was 10 mm) and is still within 0.5 s |
| 2026-11 | Step steer and limit cornering rerun on four wheels (book 11.2, and phase 5's "rerun the phase 3 tests with four wheels") | Step steer within 0.3% (peak) and 9% (settling) of the linear model. At the limit the single-track's formula misses by 3-4%: the steered fronts' drags act at +-t/2 and, unequal (the outer is loaded more), turn the car out of the corner; with that term the four-wheel form matches within 0.13% |
| 2026-11 | Telemetry schema 3 | Step 6 inserted columns into the four-wheel files instead of appending them, which by the schema rules needs a new number |
