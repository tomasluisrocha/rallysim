#pragma once
#include <algorithm>
#include "core/vec3.hpp"

namespace rallysim
{
    // One corner's suspension. This file holds both corners: the massless one (springs mode, corner_force)
    // and the one with a wheel, tyre spring, stops and optional anti-roll bar (unsprung mode, unsprung_corner_force).
    struct corner_params
    {
        Real spring_rate = 50000; // N/m: about 2.1 Hz with the ~290 kg each corner of this car carries
        Real damper_rate = 3800;  // N s/m: a damping ratio of about 0.5 on that spring
        Real rebound_ratio = 1;   // rebound damping as a multiple of damper_rate, used by the unsprung corner only;
                                  // 1 keeps the damper symmetric. A parameterisation, not a property of the car
        Real unsprung_mass = 40;  // kg: wheel, tyre, hub, brake
        Real tyre_rate = 250000;  // N/m, the tyre's vertical stiffness
        Real tyre_damping = 0;    // N s/m
        Real droop = 0.10;        // m the strut may extend past its length at rest
        Real stop_rate = 500000;  // N/m of the droop stop
        Real antiroll_rate = 0;    // N/m: anti-roll bar rate for this corner's axle, unsprung mode only
        Real motion_ratio = 1;      // strut travel per wheel travel at rest, unsprung mode only
        Real motion_ratio_gain = 0; // 1/m: the motion ratio's change per metre of wheel travel (+ bump)
        // The bump stop. Placeholder values, chosen so that no existing run reaches it (the default car's runs stay
        // below about 0.17 m); not measured values for this car.
        Real bump_travel = 0.25;      // m of compression past rest before the bump stop touches
        Real bump_stop_length = 0.03; // m over which its stiffness rises to bump_stop_rate
        Real bump_stop_rate = 500000; // N/m, its stiffness from then on
    };

    // What a corner does at one instant. Deflection is compression: positive pushes the body up.
    struct corner_load
    {
        Real deflection = 0;      // m, how far the spring is compressed (zero or less: the wheel is off the ground)
        Real deflection_rate = 0; // m/s, positive while compressing
        Real spring = 0;          // N, spring force
        Real damper = 0;          // N, damper force (positive while compressing)
        Real fz = 0;              // N, the vertical load on the tyre: spring + damper, never below zero
        Real stop = 0;            // N, the stops' force: negative from the droop stop (pulls the wheel up),
                                  // positive from the bump stop (pushes the body up); never both
        Real antiroll = 0;        // N, anti-roll bar force: positive pushes the body up and the wheel down
        Real strut = 0;           // N, spring + damper + stop + anti-roll: what the strut pushes the body up with; may pull.
                                  // Unsprung: at the wheel, the strut's own force times the motion ratio, plus the bar
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

    // The bump stop, e metres into it (k = bump_stop_rate, L = bump_stop_length). Its stiffness rises linearly from 0
    // to k over L, then stays k, so neither the force nor the stiffness jumps at contact or at L. The energy is the
    // integral of the force; the stop is elastic (no damping).
    inline Real bump_stop_force(Real e, Real k, Real L)
    {
        if (e <= 0)
            return 0;
        if (e <= L)
            return k * e * e / (2 * L);
        return k * (e - L / 2);
    }

    inline Real bump_stop_energy(Real e, Real k, Real L)
    {
        if (e <= 0)
            return 0;
        if (e <= L)
            return k * e * e * e / (6 * L);
        return k * ((e - L / 2) * (e - L / 2) / 2 + L * L / 24);
    }

    // The corner with a wheel. deflection is the strut's compression, static_deflection its compression with the
    // car at rest. The strut may pull (damper, droop stop); the tyre only pushes. The bump stop pushes once the
    // strut is bump_travel past rest. The damper is rebound_ratio times stiffer while extending.
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
        l.damper = deflection_rate >= 0 ? c.damper_rate * deflection_rate
                                        : c.rebound_ratio * c.damper_rate * deflection_rate;
        const Real e = deflection - (static_deflection - c.droop);
        l.stop = e < 0 ? c.stop_rate * e : static_cast<Real>(0);
        const Real eb = deflection - (static_deflection + c.bump_travel);
        if (eb > 0)
            l.stop += bump_stop_force(eb, c.bump_stop_rate, c.bump_stop_length);
        l.strut = l.spring + l.damper + l.stop;
        l.fz = tyre_deflection > 0
                   ? std::max(c.tyre_rate * tyre_deflection + c.tyre_damping * tyre_deflection_rate, static_cast<Real>(0))
                   : static_cast<Real>(0);
        return l;
    }
}
