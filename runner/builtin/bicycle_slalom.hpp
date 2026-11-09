#pragma once
#include "runner/run_bicycle.hpp"

namespace rallysim
{
    // Slalom, 12 s at 15 m/s: the wheel is held left, right, left, right, left, with a fast ramp
    // between them. The first and last holds are half as long as the others, so the heading swings
    // evenly to both sides of straight ahead, and the car ends pointing along x.
    inline bicycle_run bicycle_slalom()
    {
        bicycle_run r;
        r.initial.v = 15.0;
        r.dt = 0.001;
        r.steps = 12000;

        const Real d = 0.03; // steer while held (rad)
        auto key = [](Real t, Real steer)
        { return keyframe<bicycle_input>{t, bicycle_input{steer, 0.0}}; };

        r.script.keys = {key(0.0, d), key(1.35, d),     // half hold, left
                         key(1.65, -d), key(4.35, -d),  // right
                         key(4.65, d), key(7.35, d),    // left
                         key(7.65, -d), key(10.35, -d), // right
                         key(10.65, d), key(12.0, d)};  // half hold, left
        return r;
    }
}
