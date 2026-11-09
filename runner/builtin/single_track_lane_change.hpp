#pragma once
#include "runner/run_single_track.hpp"

namespace rallysim
{
    // The bicycle lane change on the dynamic model: 7 s at 15 m/s (speed held), one period of a
    // triangle-wave steer after a second of straight driving. Same steer as bicycle_lane_change, so
    // the two paths can be compared: the dynamic car lags the wheel and slides a little.
    inline single_track_run single_track_lane_change()
    {
        single_track_run r;
        r.initial = single_track_initial(15.0);
        r.hold_speed = 15.0;
        r.dt = 0.001;
        r.steps = 7000;

        const Real d = 0.02; // peak steer (rad)
        r.script.keys = {{0.0, single_track_input{0.0, 0.0}},
                         {1.0, single_track_input{0.0, 0.0}},  // straight
                         {2.0, single_track_input{d, 0.0}},    // wheel left
                         {3.0, single_track_input{0.0, 0.0}},
                         {4.0, single_track_input{-d, 0.0}},   // wheel right
                         {5.0, single_track_input{0.0, 0.0}},  // straight again
                         {7.0, single_track_input{0.0, 0.0}}};
        return r;
    }
}