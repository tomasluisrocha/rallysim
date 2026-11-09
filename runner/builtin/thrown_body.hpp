#pragma once
#include "runner/run_scripted.hpp"

namespace rallysim
{
    // Thrown body, 3 s: a force cancels gravity for the first second (straight-line flight), then
    // switches off over one step so only gravity acts. A constant torque about z spins the body up.
    inline scripted_run thrown_body()
    {
        scripted_run r;
        r.params.mass = 1200.0;
        r.params.inertia = {500.0, 1800.0, 2000.0};
        r.initial.pos = {0, 0, 1};
        r.initial.vel = {10, 0, 5};
        r.initial.omega = {0, 0, 0.5};
        r.dt = 0.001;
        r.steps = 3000;

        const wrench hover{vec3{0, 0, r.params.mass * kGravity}, vec3{0, 0, 50.0}};
        const wrench coast{vec3{0, 0, 0}, vec3{0, 0, 50.0}};
        r.script.keys = {{0.0, hover}, {0.999, hover}, {1.0, coast}};
        return r;
    }
}
