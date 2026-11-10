#pragma once
#include <algorithm>
#include "core/vec3.hpp"

namespace rallysim
{
    // One corner's suspension. This file holds both corners: the massless one (springs mode, corner_force)
    // and the one with a wheel (unsprung mode, unsprung_corner_force).
    struct corner_params
    {
        Real spring_rate = 50000; // N/m: about 2.1 Hz with the ~290 kg each corner of this car carries
        Real damper_rate = 3800;  // N s/m: a damping ratio of about 0.5 on that spring
        Real unsprung_mass = 40;  // kg: wheel, tyre, hub, brake
        Real tyre_rate = 250000;  // N/m, the tyre's vertical stiffness
        Real tyre_damping = 0;    // N s/m
        Real droop = 0.10;        // m the strut may extend past its length at rest
        Real stop_rate = 500000;  // N/m of the droop stop
    };

    // What a corner does at one instant. Deflection is compression: positive pushes the body up.
    struct corner_load
    {
        Real deflection = 0;      // m, how far the spring is compressed (zero or less: the wheel is off the ground)
        Real deflection_rate = 0; // m/s, positive while compressing
        Real spring = 0;          // N, spring force
        Real damper = 0;          // N, damper force (positive while compressing)
        Real fz = 0;              // N, the vertical load on the tyre: spring + damper, never below zero
        Real stop = 0;            // N, the droop stop's force: zero or negative (it pulls the wheel up)
        Real strut = 0;           // N, spring + damper + stop: what the strut pushes the body up with; may pull
        Real tyre_deflection = 0;      // m, the tyre's compression (zero or less: off the ground)
        Real tyre_deflection_rate = 0; // m/s
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

    // The corner with a wheel. deflection is the strut's compression, static_deflection its compression with the
    // car at rest. The strut may pull (damper, droop stop); the tyre only pushes.
    inline corner_load unsprung_corner_force(const corner_params &c, Real deflection, Real deflection_rate,
                                             Real static_deflection, Real tyre_deflection,
                                             Real tyre_deflection_rate)
    {
        corner_load l;
        l.deflection = deflection;
        l.deflection_rate = deflection_rate;
        l.tyre_deflection = tyre_deflection;
        l.tyre_deflection_rate = tyre_deflection_rate;
        l.spring = c.spring_rate * std::max(deflection, static_cast<Real>(0));
        l.damper = c.damper_rate * deflection_rate;
        const Real e = deflection - (static_deflection - c.droop);
        l.stop = e < 0 ? c.stop_rate * e : static_cast<Real>(0);
        l.strut = l.spring + l.damper + l.stop;
        l.fz = tyre_deflection > 0
                   ? std::max(c.tyre_rate * tyre_deflection + c.tyre_damping * tyre_deflection_rate, static_cast<Real>(0))
                   : static_cast<Real>(0);
        return l;
    }
}
