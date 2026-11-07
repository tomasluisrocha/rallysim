#pragma once
#include "scenarios/run_four_wheel.hpp"

namespace rallysim
{
    // The single-track lane change on the four-wheel car: 7 s at 15 m/s (speed held), one period of a
    // triangle-wave steer of 0.02 rad after a second of straight driving.
    inline four_wheel_run four_wheel_lane_change()
    {
        four_wheel_run r;
        r.initial = four_wheel_initial(r.params, 15.0);
        r.hold_speed = 15.0;
        r.dt = 0.001;
        r.steps = 7000;

        const Real d = 0.02; // peak steer (rad)
        r.script.keys = {{0.0, four_wheel_input{0.0, 0.0}},
                         {1.0, four_wheel_input{0.0, 0.0}},
                         {2.0, four_wheel_input{d, 0.0}},
                         {3.0, four_wheel_input{0.0, 0.0}},
                         {4.0, four_wheel_input{-d, 0.0}},
                         {5.0, four_wheel_input{0.0, 0.0}},
                         {7.0, four_wheel_input{0.0, 0.0}}};
        return r;
    }
}
