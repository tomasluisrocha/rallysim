#pragma once
#include <algorithm>
#include <cmath>
#include "core/keyframe_script.hpp"
#include "core/rigid_body.hpp"
#include "vehicle/load_transfer.hpp"
#include "vehicle/tyre.hpp"

namespace rallysim
{

    struct single_track_params
    {
        rigid_body_params chassis{1200, {400, 1800, 1500}}; // mass (kg), inertia about x, y, z (kg m^2)
        Real lf = 1.3;                                      // CG to front axle (m)
        Real lr = 1.2;                                      // CG to rear axle (m)
        Real front_cornering_stiffness = 60000;             // N/rad of slip angle, both front tyres together
        Real rear_cornering_stiffness = 70000;              // N/rad of slip angle, both rear tyres together
        Real friction_coefficient = 1.0;                    // peak tyre-road grip at the nominal wheel load, dimensionless
        Real load_sensitivity = 0.1;                        // fraction of the friction coefficient lost per nominal load of extra load
        Real nominal_wheel_load = 3000;                     // wheel load (N) at which the friction coefficient is friction_coefficient
        Real max_steer = 0.6;                               // steer input clamped to +-max_steer (rad)
        Real cg_height = 0.5;                               // CG height above the ground (m)
        Real track_width = 1.5;                             // distance between the left and right wheels (m)
        Real front_transfer_share = 0.5;                    // fraction of the lateral load transfer carried by the front axle
        Real wheelbase() const { return lf + lr; }
    };

    struct single_track_input
    {
        Real steer = 0;       // front wheel angle, rad (+left)
        Real drive_force = 0; // longitudinal force asked of the rear axle, N (+forward); limited by its grip
    };

    // Blend of two inputs for keyframe scripts: a at f = 0, b at f = 1.
    inline single_track_input lerp(const single_track_input &a, const single_track_input &b, Real f)
    {
        return {lerp(a.steer, b.steer, f), lerp(a.drive_force, b.drive_force, f)};
    }

    // Placeholder for low speed: slip angles divide by the forward speed, so below this value the
    // model uses it instead. Only valid well above it; handled properly in a later step.
    inline constexpr Real kMinForwardSpeed = 1.0; // m/s

    // One value per axle: slip angles (rad), vertical loads (N), grip (N) or lateral forces (N).
    struct axle_pair
    {
        Real front = 0, rear = 0;
    };

    // Vertical load on each wheel (N).
    struct wheel_loads
    {
        Real front_left = 0, front_right = 0, rear_left = 0, rear_right = 0;
    };

    inline Real applied_steer(const single_track_params &params, Real steer)
    {
        return std::clamp(steer, -params.max_steer, params.max_steer);
    }

    // Estimate of the lateral acceleration of a settled turn: forward speed times yaw rate, positive when
    // turning left. It ignores the v-dot part of a fast transient. Used for the load transfer.
    inline Real lateral_acceleration(const body_velocity &vel)
    {
        return vel.u * vel.r;
    }

    // Angle of each axle's wheels minus the angle of its velocity. Positive when the wheels point
    // further left than the axle is moving. The rear wheels are not steered.
    inline axle_pair slip_angles(const single_track_params &params, const body_velocity &vel, Real steer)
    {
        const Real u = std::max(vel.u, kMinForwardSpeed);
        return {steer - std::atan((vel.v + params.lf * vel.r) / u),
                -std::atan((vel.v - params.lr * vel.r) / u)};
    }

    // Vertical load on each axle, with the longitudinal transfer for a_x (vehicle/load_transfer.hpp).
    inline axle_pair axle_loads(const single_track_params &params, Real a_x = 0)
    {
        axle_pair a;
        longitudinal_axle_loads(params.chassis.mass, params.lf, params.lr, params.cg_height, a_x, a.front, a.rear);
        return a;
    }

    // Vertical load on each wheel: the axle loads (with the longitudinal transfer for a_x), then the lateral
    // transfer m a_y h / t from the inside wheels to the outside ones, shared between the axles by
    // front_transfer_share. In a left turn (a_y > 0) the right wheels gain. Quasi-static, no roll dynamics.
    inline wheel_loads loads_per_wheel(const single_track_params &params, Real a_x, Real a_y)
    {
        const axle_pair axle = axle_loads(params, a_x);
        const Real total = params.chassis.mass * a_y * params.cg_height / params.track_width;
        const Real front = params.front_transfer_share * total;

        wheel_loads w;
        split_axle(axle.front, front, w.front_left, w.front_right);
        split_axle(axle.rear, total - front, w.rear_left, w.rear_right);
        return w;
    }

    // Peak friction coefficient of one tyre at vertical load fz (N). Load sensitivity: it falls linearly as
    // the load rises, mu0 (1 - k (fz - fz0) / fz0), and never goes below mu0 (1 + k) / 2. Because grip is mu(fz) fz,
    // moving load from one wheel of an axle to the other lowers the axle's total grip.
    inline Real peak_friction(const single_track_params &params, Real fz)
    {
        return load_sensitive_friction(params.friction_coefficient, params.load_sensitivity, params.nominal_wheel_load, fz);
    }

    // Largest force each axle can take from the road (N): the sum over its two wheels of mu(fz) fz.
    inline axle_pair axle_grip(const single_track_params &params, const wheel_loads &w)
    {
        return {peak_friction(params, w.front_left) * w.front_left + peak_friction(params, w.front_right) * w.front_right,
                peak_friction(params, w.rear_left) * w.rear_left + peak_friction(params, w.rear_right) * w.rear_right};
    }

    // The drive force the rear axle can actually pass to the road, estimated before the loads are known.
    // With the drive on the rear axle, its load is W lf / L + F h / L, so the grip limit F <= mu (W lf + F h) / L
    // gives F <= mu W lf / (L - mu h) when accelerating and |F| <= mu W lf / (L + mu h) when braking
    // (the classic rear-drive traction limit, at the nominal friction coefficient). Never more than mu W.
    inline Real traction_limited_force(const single_track_params &params, Real drive_force)
    {
        const Real mu = params.friction_coefficient, h = params.cg_height, L = params.wheelbase();
        const Real weight = params.chassis.mass * kGravity;
        const Real all = mu * weight;
        const Real forward = (L - mu * h > 0) ? std::min(mu * weight * params.lf / (L - mu * h), all) : all;
        const Real backward = mu * weight * params.lf / (L + mu * h);
        return std::clamp(drive_force, -backward, forward);
    }

    // Estimate of the longitudinal acceleration, used for the load transfer: the traction-limited drive
    // force over the mass. It ignores drag and the longitudinal part of the steered front force.
    inline Real longitudinal_acceleration(const single_track_params &params, const single_track_input &input)
    {
        return traction_limited_force(params, input.drive_force) / params.chassis.mass;
    }

    // Sideways force on each axle (axle frame, +left): linear in slip angle, capped by that axle's grip (N).
    // A positive slip angle gives a positive force, which reduces the slip.
    inline axle_pair lateral_forces(const single_track_params &params, const axle_pair &slip, const axle_pair &grip)
    {
        return {std::clamp(params.front_cornering_stiffness * slip.front, -grip.front, grip.front),
                std::clamp(params.rear_cornering_stiffness * slip.rear, -grip.rear, grip.rear)};
    }

    // Everything the tyres do in one step, computed once and shared by the step and the telemetry.
    struct single_track_forces
    {
        Real steer = 0;      // front wheel angle used (rad), the input limited to max_steer
        body_velocity vel;   // velocity of the CG in the car's frame, and the yaw rate
        Real a_x = 0, a_y = 0; // accelerations (m/s^2) used for the load transfer
        axle_pair slip;      // slip angles (rad)
        wheel_loads fz;      // vertical load on each wheel (N)
        axle_pair grip;      // largest force each axle can take (N), with load sensitivity
        Real fx_rear = 0;    // longitudinal force of the rear axle (N), the drive force limited by its grip
        axle_pair fy;        // lateral forces (N), axle frame; the rear limit is what the drive force leaves
    };

    // Order: steer and velocity, loads (from the estimated accelerations), grip, slip, then forces. The
    // rear axle shares its grip between drive and side force (friction circle): sqrt(grip^2 - fx^2) is left
    // for the side force.
    inline single_track_forces evaluate(const rigid_body_state &state, const single_track_params &params,
                                        const single_track_input &input)
    {
        single_track_forces f;
        f.steer = applied_steer(params, input.steer);
        f.vel = velocity_in_body(state);
        f.a_x = longitudinal_acceleration(params, input);
        f.a_y = lateral_acceleration(f.vel);
        f.fz = loads_per_wheel(params, f.a_x, f.a_y);
        f.grip = axle_grip(params, f.fz);
        f.slip = slip_angles(params, f.vel, f.steer);
        f.fx_rear = std::clamp(input.drive_force, -f.grip.rear, f.grip.rear);
        const axle_pair side_grip{f.grip.front, std::sqrt(f.grip.rear * f.grip.rear - f.fx_rear * f.fx_rear)};
        f.fy = lateral_forces(params, f.slip, side_grip);
        return f;
    }

    // Forces and torque on the car from its tyres, in the frames the rigid body expects.
    inline wrench single_track_wrench(const rigid_body_state &state, const single_track_params &params,
                                      const single_track_input &input)
    {
        const single_track_forces f = evaluate(state, params, input);

        // body frame: the front force is sideways to the steered wheel, the drive force is on the rear axle
        const vec3 force_front{-f.fy.front * std::sin(f.steer), f.fy.front * std::cos(f.steer), 0};
        const vec3 force_rear{f.fx_rear, f.fy.rear, 0};

        wrench w{};
        add_force_at(w, state.q, to_world(state.q, force_front), vec3{params.lf, 0, 0});
        add_force_at(w, state.q, to_world(state.q, force_rear), vec3{-params.lr, 0, 0});
        w.force.z += params.chassis.mass * kGravity; // cancels gravity: the car stays on the ground plane for now
        return w;
    }

    inline void step(rigid_body_state &state, const single_track_params &params, const single_track_input &input, Real dt)
    {
        step(state, params.chassis, single_track_wrench(state, params, input), dt);
    }
}
