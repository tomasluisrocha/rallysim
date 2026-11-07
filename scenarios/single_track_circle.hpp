#pragma once
#include "scenarios/run_single_track.hpp"

namespace rallysim
{
    // Steer step into a steady circle, 20 s at 6 m/s (speed held): straight for a second, then the
    // wheel goes to 0.2 rad over 0.1 s and stays. The yaw rate settles within about a second on a
    // circle of about 66 m radius (a_lat about 3.4 m/s^2), well damped with no visible overshoot, so
    // the rest of the run is the steady state that the understeer gradient describes.
    inline single_track_run single_track_circle()
    {
        single_track_run r;
        r.initial = single_track_initial(6.0);
        r.hold_speed = 6.0;
        r.dt = 0.001;
        r.steps = 20000;

        const Real steer = 0.2;
        r.script.keys = {{0.0, single_track_input{0.0, 0.0}},
                         {1.0, single_track_input{0.0, 0.0}},
                         {1.1, single_track_input{steer, 0.0}},
                         {20.0, single_track_input{steer, 0.0}}};
        return r;
    }
}