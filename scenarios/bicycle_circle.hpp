#pragma once
#include <cmath>
#include "scenarios/run_bicycle.hpp"

namespace rallysim
{
    // Constant-steer circle, one lap: 6 m/s with 0.2 rad of steer. The CG runs on a circle of radius
    // sqrt(lr^2 + (L / tan(steer))^2) = 12.39 m, so one lap takes 2 pi R / v = 12.97 s, and the car
    // ends back at its start (to within the step: the lap is rounded to a whole number of steps).
    inline bicycle_run bicycle_circle()
    {
        bicycle_run r;
        r.initial.v = 6.0;
        r.dt = 0.001;

        const Real steer = 0.2;
        const Real radius = std::hypot(r.params.lr, r.params.wheelbase() / std::tan(steer));
        r.steps = std::lround(2.0 * 3.14159265358979323846 * radius / r.initial.v / r.dt);

        r.script.keys = {{0.0, bicycle_input{steer, 0.0}}};
        return r;
    }
}
