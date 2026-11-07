#pragma once
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