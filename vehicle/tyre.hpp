#pragma once
#include <algorithm>
#include <cmath>
#include "core/vec3.hpp"

namespace rallysim
{
    // One tyre (book section 7.2, rungs 1 to 3): linear in slip, capped by load-sensitive friction, with
    // combined slip shared on a friction circle. Phase 5 replaces this file with a tyre that has a peak and
    // a fall-off; the rest of the car only calls tyre_forces().
    struct tyre_params
    {
        Real longitudinal_stiffness = 90000; // N per unit of slip ratio (force at 100% slip if it stayed linear)
        Real cornering_stiffness = 30000;    // N per rad of slip angle, one tyre
        Real friction_coefficient = 1.0;     // peak grip at the nominal load, dimensionless
        Real load_sensitivity = 0.1;         // fraction of the friction coefficient lost per nominal load of extra load
        Real nominal_load = 3000;            // N, the load at which the friction coefficient is friction_coefficient
        Real relaxation_length_x = 0.2;      // m rolled for the longitudinal force to build up (63%)
        Real relaxation_length_y = 0.4;      // m rolled for the lateral force to build up (63%)
        Real damping_x = 18000;              // N s/m, low-speed damping of the contact patch along the wheel
        Real damping_y = 9500;               // N s/m, low-speed damping of the contact patch across the wheel
        Real damping_fade_speed = 2.5;       // m/s, the damping fades out linearly by this rolling speed
    };

    // Force of one tyre on the car, in the wheel frame (N): x along the wheel, y to its left.
    struct tyre_force
    {
        Real fx = 0, fy = 0;
    };

    // Peak friction coefficient at vertical load fz (N): mu0 (1 - k (fz - fz0) / fz0), never below zero.
    // Shared with the single-track model, so both use the same arithmetic.
    inline Real load_sensitive_friction(Real mu0, Real sensitivity, Real nominal_load, Real fz)
    {
        const Real drop = sensitivity * (fz - nominal_load) / nominal_load;
        return mu0 * std::max(static_cast<Real>(1) - drop, static_cast<Real>(0));
    }

    inline Real peak_friction(const tyre_params &t, Real fz)
    {
        return load_sensitive_friction(t.friction_coefficient, t.load_sensitivity, t.nominal_load, fz);
    }

    // Largest force the tyre can take from the road at load fz (N): mu(fz) fz, zero when unloaded.
    inline Real tyre_grip(const tyre_params &t, Real fz)
    {
        return fz > 0 ? peak_friction(t, fz) * fz : static_cast<Real>(0);
    }

    // Tyre force from the slip ratio (dimensionless) and the lateral slip (tangent of the slip angle, which is
    // the slip angle for small angles) at vertical load fz (N).
    //
    // Each direction is linear in its own slip. If the combined force is more than the grip, both parts are
    // scaled down together, so the force keeps its direction and its size is exactly the grip (friction
    // circle). Positive slip gives positive force, which reduces the slip.
    inline tyre_force tyre_forces(const tyre_params &t, Real slip_ratio, Real lateral_slip, Real fz)
    {
        const Real grip = tyre_grip(t, fz);
        if (grip <= 0)
            return {};
        tyre_force f{t.longitudinal_stiffness * slip_ratio, t.cornering_stiffness * lateral_slip};
        const Real size = std::hypot(f.fx, f.fy);
        if (size > grip)
        {
            const Real scale = grip / size;
            f.fx *= scale;
            f.fy *= scale;
        }
        return f;
    }
}
