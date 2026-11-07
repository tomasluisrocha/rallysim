#pragma once
#include "scenarios/run_scripted.hpp"

namespace rallysim
{
    // Lane change, 9 s: a body coasting along x at 5 m/s is pushed left, then right, so it ends
    // 11.25 m to the left, still moving along x. In plan view the path is an S shape.
    // The z force cancels gravity throughout and there is no torque, so the heading never changes.
    inline scripted_run driven_body()
    {
        scripted_run r;
        r.params.mass = 1200.0;
        r.params.inertia = {500.0, 1800.0, 2000.0};
        r.initial.vel = {5, 0, 0};
        r.dt = 0.001;
        r.steps = 9000;

        const Real lift = r.params.mass * kGravity;
        auto key = [&](Real t, Real fy)
        { return keyframe{t, wrench{vec3{0, fy, lift}, vec3{}}}; };

        r.script.keys = {key(0.0, 0),
                         key(1.0, 0),      // straight
                         key(2.0, 1500),   // ramp up: push left
                         key(4.0, 1500),   // hold
                         key(5.0, -1500),  // swing to push right
                         key(7.0, -1500),  // hold
                         key(8.0, 0),      // ramp off
                         key(9.0, 0)};     // straight
        return r;
    }
}