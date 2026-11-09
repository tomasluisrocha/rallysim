#pragma once
#include <algorithm>
#include "core/vec3.hpp"

namespace rallysim
{
    // One corner's suspension (phase 4, step 6): a vertical spring and damper between the body and the ground
    // under its wheel. A placeholder until phase 6, which adds the wheel's own mass, the tyre's own
    // spring, bump stops, anti-roll bars and the suspension geometry; only this file changes then.
    struct corner_params
    {
        Real spring_rate = 50000; // N/m: about 2.1 Hz with the ~290 kg each corner of this car carries
        Real damper_rate = 3800;  // N s/m: a damping ratio of about 0.5 on that spring
    };

    // What a corner does at one instant. Deflection is compression: positive pushes the body up.
    struct corner_load
    {
        Real deflection = 0;      // m, how far the spring is compressed (zero or less: the wheel is off the ground)
        Real deflection_rate = 0; // m/s, positive while compressing
        Real spring = 0;          // N, spring force
        Real damper = 0;          // N, damper force (positive while compressing)
        Real fz = 0;              // N, the vertical load on the tyre: spring + damper, never below zero
    };

    // The load from the corner's compression. The suspension can push the body up but never pull it down: a
    // corner that is not compressed carries nothing (the wheel has lifted), and a damper that would pull harder
    // than the spring pushes leaves the tyre unloaded instead of sucking it onto the ground.
    inline corner_load corner_force(const corner_params &c, Real deflection, Real deflection_rate)
    {
        corner_load l;
        l.deflection = deflection;
        l.deflection_rate = deflection_rate;
        if (deflection <= 0)
            return l;
        l.spring = c.spring_rate * deflection;
        l.damper = c.damper_rate * deflection_rate;
        l.fz = std::max(l.spring + l.damper, static_cast<Real>(0));
        return l;
    }
}
