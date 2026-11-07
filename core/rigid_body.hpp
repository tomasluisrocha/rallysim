#pragma once
#include <algorithm>
#include "core/quat.hpp"
#include "core/wrench.hpp"

namespace rallysim
{

    struct rigid_body_state
    {
        vec3 pos;   // m, world frame
        vec3 vel;   // m/s, world frame
        quat q;     // body -> world
        vec3 omega; // rad/s, body frame
    };

    struct rigid_body_params
    {
        Real mass = 1;         // kg
        vec3 inertia{1, 1, 1}; // kg m^2, principal moments about body x, y, z
    };

    inline constexpr Real kGravity = 9.81; // m/s^2, acts along -z

    // Velocity of the CG in the body's own frame, and the yaw rate: what a ground vehicle cares about.
    struct body_velocity
    {
        Real u = 0; // forward, m/s
        Real v = 0; // sideways (+left), m/s
        Real r = 0; // yaw rate, rad/s
    };

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

    // Body roll and pitch (rad), as yaw-pitch-roll angles with ISO 8855 signs: roll is about the forward axis,
    // positive with the right side down (a car leaning out of a left turn); pitch is about the left axis,
    // positive nose down (a car diving under braking).
    inline Real roll_angle(const quat &q)
    {
        return std::atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y));
    }

    inline Real pitch_angle(const quat &q)
    {
        return std::asin(std::clamp(2 * (q.w * q.y - q.z * q.x), static_cast<Real>(-1), static_cast<Real>(1)));
    }

    // Advances the body by one fixed time step dt (s).
    inline void step(rigid_body_state &s, const rigid_body_params &p, const wrench &w, Real dt)
    {
        // Linear motion: semi-implicit Euler (velocity first, then position).
        const vec3 gravity{0, 0, -kGravity};
        const vec3 acceleration = w.force / p.mass + gravity;
        s.vel += acceleration * dt;
        s.pos += s.vel * dt;

        // Angular motion: Euler's equations for principal axes,
        //   I * omega_dot = torque - omega x (I * omega)
        const vec3 angular_momentum{p.inertia.x * s.omega.x,
                                    p.inertia.y * s.omega.y,
                                    p.inertia.z * s.omega.z};
        const vec3 net_torque = w.torque - cross(s.omega, angular_momentum);
        const vec3 angular_acceleration{net_torque.x / p.inertia.x,
                                        net_torque.y / p.inertia.y,
                                        net_torque.z / p.inertia.z};
        s.omega += angular_acceleration * dt;

        // Orientation: integrate the new body-frame angular velocity.
        s.q = integrate(s.q, s.omega, dt);
    }
}
