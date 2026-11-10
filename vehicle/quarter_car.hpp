#pragma once
#include <algorithm>
#include <cmath>
#include "core/rigid_body.hpp"

namespace rallysim
{
    // One corner, vertical only: a sprung mass on a spring and damper, on a wheel mass, on a tyre spring, on a road.
    struct quarter_car_params
    {
        Real sprung_mass = 260;   // kg
        Real unsprung_mass = 40;  // kg
        Real spring_rate = 50000; // N/m
        Real damper_rate = 3800;  // N s/m
        Real tyre_rate = 250000;  // N/m
        Real tyre_damping = 0;    // N s/m
    };

    // m and m/s, + up. zs and zu are measured from where each mass sits with both springs unloaded on a road at
    // height 0: spring compression is zu - zs, tyre compression is road - zu.
    struct quarter_car_state
    {
        Real zs = 0, vs = 0, zu = 0, vu = 0;
    };

    struct quarter_car_forces
    {
        Real spring = 0; // N
        Real damper = 0; // N
        Real strut = 0;  // N, spring + damper; + pushes the body up, may pull
        Real tyre = 0;   // N, pushes, never pulls
    };

    inline quarter_car_forces evaluate(const quarter_car_state &s, const quarter_car_params &p, Real road, Real road_rate)
    {
        quarter_car_forces f;
        f.spring = p.spring_rate * (s.zu - s.zs);
        f.damper = p.damper_rate * (s.vu - s.vs);
        f.strut = f.spring + f.damper;
        f.tyre = std::max(p.tyre_rate * (road - s.zu) + p.tyre_damping * (road_rate - s.vu), static_cast<Real>(0));
        return f;
    }

    // Forces from the state at the start, velocities first, then positions.
    inline void step(quarter_car_state &s, const quarter_car_params &p, Real road, Real road_rate, Real dt)
    {
        const quarter_car_forces f = evaluate(s, p, road, road_rate);
        s.vs += (f.strut / p.sprung_mass - kGravity) * dt;
        s.vu += ((f.tyre - f.strut) / p.unsprung_mass - kGravity) * dt;
        s.zs += s.vs * dt;
        s.zu += s.vu * dt;
    }

    // At rest on a road at height `road`.
    inline quarter_car_state quarter_car_rest(const quarter_car_params &p, Real road)
    {
        quarter_car_state s;
        s.zu = road - (p.sprung_mass + p.unsprung_mass) * kGravity / p.tyre_rate;
        s.zs = s.zu - p.sprung_mass * kGravity / p.spring_rate;
        return s;
    }

    // J: kinetic + gravity + spring + tyre (no tyre energy when the wheel is off the road).
    inline Real quarter_car_energy(const quarter_car_state &s, const quarter_car_params &p, Real road)
    {
        const Real dz = s.zs - s.zu;
        const Real tyre = std::max(road - s.zu, static_cast<Real>(0));
        return Real(0.5) * p.sprung_mass * s.vs * s.vs + Real(0.5) * p.unsprung_mass * s.vu * s.vu +
               p.sprung_mass * kGravity * s.zs + p.unsprung_mass * kGravity * s.zu +
               Real(0.5) * p.spring_rate * dz * dz + Real(0.5) * p.tyre_rate * tyre * tyre;
    }

    // The first field of the state that is not a finite number, or nullptr if all are.
    inline const char *non_finite_field(const quarter_car_state &s)
    {
        if (!std::isfinite(s.zs))
            return "zs";
        if (!std::isfinite(s.vs))
            return "vs";
        if (!std::isfinite(s.zu))
            return "zu";
        if (!std::isfinite(s.vu))
            return "vu";
        return nullptr;
    }
}
