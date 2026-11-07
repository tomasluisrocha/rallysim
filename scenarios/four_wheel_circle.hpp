#pragma once
#include "scenarios/run_four_wheel.hpp"

namespace rallysim
{
    // The single-track circle on the four-wheel car: 20 s at 6 m/s (speed held), straight for a second, then
    // the wheel goes to 0.2 rad over 0.1 s and stays. Same car numbers, so the two can be overlaid.
    inline four_wheel_run four_wheel_circle()
    {
        four_wheel_run r;
        r.initial = four_wheel_initial(r.params, 6.0);
        r.hold_speed = 6.0;
        r.dt = 0.001;
        r.steps = 20000;

        const Real steer = 0.2;
        r.script.keys = {{0.0, four_wheel_input{0.0, 0.0}},
                         {1.0, four_wheel_input{0.0, 0.0}},
                         {1.1, four_wheel_input{steer, 0.0}},
                         {20.0, four_wheel_input{steer, 0.0}}};
        return r;
    }
}
