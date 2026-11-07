#pragma once
#include "scenarios/run_bicycle.hpp"

namespace rallysim
{
    // Lane change, 7 s at 15 m/s: one period of a triangle-wave steer (left, then right) after a
    // second of straight driving. The heading is the integral of the steer, so it rises while the
    // wheel is left, falls back while it is right, and is zero again at the end. The car finishes
    // pointing along x, a few metres to the left of where it started.
    inline bicycle_run bicycle_lane_change()
    {
        bicycle_run r;
        r.initial.v = 15.0;
        r.dt = 0.001;
        r.steps = 7000;

        const Real d = 0.02; // peak steer (rad)
        r.script.keys = {{0.0, bicycle_input{0.0, 0.0}},
                         {1.0, bicycle_input{0.0, 0.0}},  // straight
                         {2.0, bicycle_input{d, 0.0}},    // wheel left
                         {3.0, bicycle_input{0.0, 0.0}},
                         {4.0, bicycle_input{-d, 0.0}},   // wheel right
                         {5.0, bicycle_input{0.0, 0.0}},  // straight again
                         {7.0, bicycle_input{0.0, 0.0}}};
        return r;
    }
}