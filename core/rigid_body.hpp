#pragma once
#include <algorithm>
#include "core/quat.hpp"
#include "core/wrench.hpp"

namespace rallysim
{

    // The one stored motion of a body; every other velocity (body_velocity, a wheel's patch velocity) is worked out
    // from it. The frames are mixed on purpose, the usual choice for a rigid body: the velocity is in the world
    // frame, where Newton's law with gravity (a constant world vector) is simplest, and the angular velocity is in
    // the body frame, where the inertia is constant and Euler's equations are simplest.
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

    // The body's motion in its own axes, the six ISO 8855 vehicle velocities: u, v, w along the body's x (forward),
    // y (left) and z (up) axes, and p, q, r the roll, pitch and yaw rates about them. Body axes tilt with the body,
    // so w is not the rate of climb: a car going straight on level ground with its nose pitched down by theta has
    // w = u tan(theta), with no vertical motion at all. The world vertical speed (heave) is the state's vel.z.
    //
    // Never built by position: with six numbers of the same type, {a, b, c} would silently fill whichever fields come
    // first, and reordering the fields would change what every such line means. So the fields are set by name
    // (velocity_in_body), and the one shorthand is the constructor for the planar three, body_velocity(u, v, r),
    // which most of the code needs (slip angles, sideslip, the controllers). Having a constructor also stops C++
    // from accepting a positional {...} list for the fields at all.
    struct body_velocity
    {
        Real u = 0; // forward, m/s
        Real v = 0; // sideways (+left), m/s
        Real w = 0; // along the body's z axis (+up), m/s
        Real p = 0; // roll rate, rad/s (+ right side going down, the sense of roll_angle)
        Real q = 0; // pitch rate, rad/s (+ nose going down, the sense of pitch_angle)
        Real r = 0; // yaw rate, rad/s (+ turning left)

        body_velocity() = default;
        // the planar motion: forward and sideways speed and the yaw rate, the rest zero
        body_velocity(Real forward, Real sideways, Real yaw_rate) : u(forward), v(sideways), r(yaw_rate) {}
    };

    // The state's world velocity turned into the body's axes, and its body-frame angular velocity: all six.
    inline body_velocity velocity_in_body(const rigid_body_state &state)
    {
        const vec3 v = to_body(state.q, state.vel);
        body_velocity b;
        b.u = v.x;
        b.v = v.y;
        b.w = v.z;
        b.p = state.omega.x;
        b.q = state.omega.y;
        b.r = state.omega.z;
        return b;
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

    // The first field of the state that is not a finite number (NaN or infinity), or nullptr if all are. The
    // runners stop on it through require_finite (core/finite.hpp: fail on any non-finite state).
    inline const char *non_finite_field(const rigid_body_state &s)
    {
        if (!is_finite(s.pos))
            return "body.pos";
        if (!is_finite(s.vel))
            return "body.vel";
        if (!is_finite(s.q))
            return "body.q";
        if (!is_finite(s.omega))
            return "body.omega";
        return nullptr;
    }
}
