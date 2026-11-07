#pragma once
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
}
