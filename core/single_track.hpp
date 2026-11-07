#pragma once
#include <algorithm>
#include <cmath>
#include "core/rigid_body.hpp"

namespace rallysim
{

    struct single_track_params
    {
        rigid_body_params chassis{1200, {400, 1800, 1500}}; // mass (kg), inertia about x, y, z (kg m^2)
        Real lf = 1.3;                                      // CG to front axle (m)
        Real lr = 1.2;                                      // CG to rear axle (m)
        Real front_cornering_stiffness = 60000;             // N/rad of slip angle, both front tyres together
        Real rear_cornering_stiffness = 70000;              // N/rad of slip angle, both rear tyres together
        Real friction_coefficient = 1.0;                    // peak tyre-road grip, dimensionless
        Real max_steer = 0.6;                               // steer input clamped to +-max_steer (rad)
        Real wheelbase() const { return lf + lr; }
    };

    struct single_track_input
    {
        Real steer = 0;       // front wheel angle, rad (+left)
        Real drive_force = 0; // total longitudinal force from the road on the car, N (+forward)
    };

    // Placeholder for low speed: slip angles divide by the forward speed, so below this value the
    // model uses it instead. Only valid well above it; handled properly in a later step.
    inline constexpr Real kMinForwardSpeed = 1.0; // m/s

    // One value per axle: slip angles (rad), vertical loads (N) or lateral forces (N).
    struct axle_pair
    {
        Real front = 0, rear = 0;
    };

    // Velocity of the CG in the car's own frame, and the yaw rate.
    struct body_velocity
    {
        Real u = 0; // forward, m/s
        Real v = 0; // sideways (+left), m/s
        Real r = 0; // yaw rate, rad/s
    };

    inline Real applied_steer(const single_track_params &params, Real steer)
    {
        return std::clamp(steer, -params.max_steer, params.max_steer);
    }

    inline body_velocity velocity_in_body(const rigid_body_state &state)
    {
        const vec3 v = to_body(state.q, state.vel);
        return {v.x, v.y, state.omega.z};
    }

    // Direction of the body x axis on the ground, counter-clockwise from world +x (rad).
    inline Real yaw_angle(const quat &q)
    {
        return std::atan2(2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.y * q.y + q.z * q.z));
    }

    // Angle of each axle's wheels minus the angle of its velocity. Positive when the wheels point
    // further left than the axle is moving. The rear wheels are not steered.
    inline axle_pair slip_angles(const single_track_params &params, const body_velocity &vel, Real steer)
    {
        const Real u = std::max(vel.u, kMinForwardSpeed);
        return {steer - std::atan((vel.v + params.lf * vel.r) / u),
                -std::atan((vel.v - params.lr * vel.r) / u)};
    }

    // Static weight on each axle: the CG is closer to the axle that carries more.
    inline axle_pair axle_loads(const single_track_params &params)
    {
        const Real weight = params.chassis.mass * kGravity;
        return {weight * params.lr / params.wheelbase(), weight * params.lf / params.wheelbase()};
    }

    // Sideways force on each axle (axle frame, +left): linear in slip angle, capped by friction.
    // A positive slip angle gives a positive force, which reduces the slip.
    inline axle_pair lateral_forces(const single_track_params &params, const axle_pair &slip, const axle_pair &loads)
    {
        const Real limit_f = params.friction_coefficient * loads.front;
        const Real limit_r = params.friction_coefficient * loads.rear;
        return {std::clamp(params.front_cornering_stiffness * slip.front, -limit_f, limit_f),
                std::clamp(params.rear_cornering_stiffness * slip.rear, -limit_r, limit_r)};
    }

    // Forces and torque on the car from its tyres, in the frames the rigid body expects.
    inline wrench single_track_wrench(const rigid_body_state &state, const single_track_params &params,
                                      const single_track_input &input)
    {
        const Real steer = applied_steer(params, input.steer);
        const axle_pair fy = lateral_forces(params, slip_angles(params, velocity_in_body(state), steer),
                                            axle_loads(params));

        // body frame: the front force is sideways to the steered wheel, the drive force is on the rear axle
        const vec3 force_front{-fy.front * std::sin(steer), fy.front * std::cos(steer), 0};
        const vec3 force_rear{input.drive_force, fy.rear, 0};

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