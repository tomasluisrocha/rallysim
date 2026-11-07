#pragma once
#include <cmath>
#include <cstddef>
#include <vector>
#include "scenarios/run_single_track.hpp"

// Helpers shared by the single-track tests. Only test setup and closed-form oracles live here; the
// model itself is in vehicle/single_track.hpp.

namespace rallysim
{
    // A car on the ground plane at (x, y) with heading `yaw`, velocity (u, v) in its own frame and yaw rate r.
    inline rigid_body_state car(Real x, Real y, Real yaw, Real u, Real v, Real r)
    {
        rigid_body_state s;
        s.pos = vec3{x, y, 0};
        s.q = from_axis_angle(vec3{0, 0, 1}, yaw);
        s.vel = to_world(s.q, vec3{u, v, 0});
        s.omega = vec3{0, 0, r};
        return s;
    }

    // The same car at the origin.
    inline rigid_body_state car(Real yaw, Real u, Real v, Real r)
    {
        return car(0, 0, yaw, u, v, r);
    }

    // Steady-state understeer gradient of the car, rad per m/s^2 of lateral acceleration:
    // K = (Wf / Cf - Wr / Cr) / g, from the static axle loads.
    inline Real understeer_gradient(const single_track_params &p)
    {
        const axle_pair w = axle_loads(p);
        return (w.front / p.front_cornering_stiffness - w.rear / p.rear_cornering_stiffness) / kGravity;
    }

    // A run at constant speed (held by the speed controller) and constant steer from the start.
    inline single_track_run constant_steer(const single_track_params &p, Real speed, Real steer, long steps)
    {
        single_track_run r;
        r.params = p;
        r.initial = single_track_initial(speed);
        r.hold_speed = speed;
        r.script.keys = {{0.0, single_track_input{steer, 0.0}}};
        r.steps = steps;
        return r;
    }

    // The same response from the linearised equations in (v, r) at constant forward speed u:
    //   v' = -(Cf + Cr) / (m u) v - (u + (Cf lf - Cr lr) / (m u)) r + Cf / m steer
    //   r' = -(Cf lf - Cr lr) / (Iz u) v - (Cf lf^2 + Cr lr^2) / (Iz u) r + Cf lf / Iz steer
    // integrated with fourth-order Runge-Kutta, which is exact for this purpose at a 1 ms step. It
    // shares no code with the simulation, so it is an independent oracle for small steer.
    inline std::vector<Real> linear_yaw_rate(const single_track_params &p, Real u, Real steer, Real dt, long steps)
    {
        const Real m = p.chassis.mass, iz = p.chassis.inertia.z;
        const Real cf = p.front_cornering_stiffness, cr = p.rear_cornering_stiffness;
        const Real a11 = -(cf + cr) / (m * u);
        const Real a12 = -u - (cf * p.lf - cr * p.lr) / (m * u);
        const Real a21 = -(cf * p.lf - cr * p.lr) / (iz * u);
        const Real a22 = -(cf * p.lf * p.lf + cr * p.lr * p.lr) / (iz * u);
        const Real b1 = cf / m * steer, b2 = cf * p.lf / iz * steer;

        auto derivative = [&](Real v, Real r, Real &dv, Real &dr)
        {
            dv = a11 * v + a12 * r + b1;
            dr = a21 * v + a22 * r + b2;
        };

        Real v = 0, r = 0;
        std::vector<Real> out{r};
        for (long i = 0; i < steps; ++i)
        {
            Real k1v, k1r, k2v, k2r, k3v, k3r, k4v, k4r;
            derivative(v, r, k1v, k1r);
            derivative(v + 0.5 * dt * k1v, r + 0.5 * dt * k1r, k2v, k2r);
            derivative(v + 0.5 * dt * k2v, r + 0.5 * dt * k2r, k3v, k3r);
            derivative(v + dt * k3v, r + dt * k3r, k4v, k4r);
            v += dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v);
            r += dt / 6 * (k1r + 2 * k2r + 2 * k3r + k4r);
            out.push_back(r);
        }
        return out;
    }

    // What the book asks of a steering step: overshoot, peak time and settling time of the yaw rate.
    struct step_response
    {
        Real final = 0;     // value at the end of the record
        Real overshoot = 0; // peak / final - 1
        Real t_peak = 0;    // s
        Real t_settle = 0;  // s, after this the yaw rate stays within 5% of the final value
    };

    inline step_response measure(const std::vector<Real> &r, Real dt)
    {
        step_response m;
        m.final = r.back();
        std::size_t peak = 0, last_outside = 0;
        for (std::size_t i = 0; i < r.size(); ++i)
        {
            if (r[i] > r[peak])
                peak = i;
            if (std::fabs(r[i] - m.final) > 0.05 * std::fabs(m.final))
                last_outside = i;
        }
        m.overshoot = r[peak] / m.final - 1.0;
        m.t_peak = static_cast<Real>(peak) * dt;
        m.t_settle = static_cast<Real>(last_outside) * dt;
        return m;
    }
}
