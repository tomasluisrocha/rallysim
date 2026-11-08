#pragma once
#include <algorithm>
#include <cmath>
#include "core/vec3.hpp"

namespace rallysim
{
    // One tyre (book section 7.2). Rungs 1 to 3: linear in slip, capped by load-sensitive friction, with
    // combined slip shared on a friction circle. Phase 5 adds a curve with a peak and a fall-off (rungs 4 and 5);
    // the rest of the car only calls tyre_forces(), whichever curve is chosen.

    // Shape of the force against slip.
    enum class tyre_curve
    {
        linear,    // straight line of slope C, flat at the grip: no peak, no fall-off (the closed-form tests use it)
        peak_slide // rises with slope C, peaks smoothly, falls to the sliding force (brush when peak_slip = 3, sliding_ratio = 1)
    };

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

        // The peak_slide curve. Its slips are normalised: slip 1 is where the linear tyre would reach the grip,
        // C s = mu(Fz) Fz, so the peak moves to larger slips as the load grows (as a brush tyre's does).
        tyre_curve curve = tyre_curve::linear;
        Real peak_slip = 2.0;     // normalised slip at the peak force; between 1.5 and 3. Above 3 the rise overshoots
                                  // the grip; below 1.5 it bulges above the linear tyre, so braking while cornering
                                  // would add side force (the gate test checks the range)
        Real sliding_slip = 4.0;  // normalised slip from which the tyre slides fully; above peak_slip
        Real sliding_ratio = 0.8; // sliding force over peak force (0 to 1); 1 gives no fall-off
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

    // The peak_slide curve: force over grip (0 to 1) at normalised slip s, where s >= 0 is the size of the
    // normalised slip (the caller applies the direction) and s = 1 is where the linear tyre would reach the grip.
    // A cubic rises from 0 with slope 1 to 1 at peak_slip, a half-cosine falls to sliding_ratio at sliding_slip,
    // and it stays there. Both joins are flat, so the slope never jumps. Assumes the ranges in tyre_params.
    inline Real curve_shape(const tyre_params &t, Real s)
    {
        const Real p = t.peak_slip;
        const Real q = t.sliding_slip;
        const Real r = t.sliding_ratio;

        if (s <= p)
        {
            // cubic s + a s^2 + b s^3: slope 1 at 0 (built in), value 1 and slope 0 at the peak.
            const Real a = (3 - 2 * p) / (p * p);
            const Real b = (p - 2) / (p * p * p);
            return s + a * s * s + b * s * s * s;
        }
        if (s < q)
        {
            const Real u = (s - p) / (q - p);                 // 0 at the peak, 1 at full sliding
            return r + (1 - r) * (1 + std::cos(kPi * u)) / 2; // 1 at u = 0, r at u = 1, flat at both ends
        }
        return r;
    }

    // Largest normalised slip the relaxed slips may build up to (cap_slips in wheel.hpp). A tyre cannot hold more
    // twist than its force needs: the linear force is flat from 1 (it reaches the grip there), the peak_slide
    // force from sliding_slip (it slides fully there). Capping there leaves the force unchanged but stops a
    // sliding tyre storing a twist it would release after it stops.
    inline Real max_normalised_slip(const tyre_params &t)
    {
        switch (t.curve)
        {
        case tyre_curve::linear:
            return 1; // the linear force reaches the grip at s = 1
        case tyre_curve::peak_slide:
            return t.sliding_slip; // past full sliding the curve is flat
        }
        return 1; // not reached: every curve returns above
    }

    // Tyre force from the slip ratio (dimensionless) and the lateral slip (tangent of the slip angle, which is
    // the slip angle for small angles) at vertical load fz (N).
    //
    // linear: each direction is linear in its own slip. If the combined force is more than the grip, both parts
    // are scaled down together, so the force keeps its direction and its size is exactly the grip (friction
    // circle).
    //
    // peak_slide (book section 17.8): each slip is normalised by its own stiffness (1 is where the linear tyre
    // would reach the grip), the size of the combined slip is looked up once on curve_shape, and the force is
    // split along the direction of the slip. So the two directions share one grip, a locked or spinning wheel
    // loses its side force, and at small slip the force is the linear tyre's.
    //
    // Both: positive slip gives positive force, which reduces the slip; never more than the grip.
    inline tyre_force tyre_forces(const tyre_params &t, Real slip_ratio, Real lateral_slip, Real fz)
    {
        const Real grip = tyre_grip(t, fz);
        if (grip <= 0)
            return {}; // unloaded: no force, whatever the curve

        switch (t.curve)
        {
        case tyre_curve::linear:
        {
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
        case tyre_curve::peak_slide:
        {
            // Normalise each slip by its stiffness: 1 is where the linear tyre would reach the grip (book section 17.8).
            const Real kx = t.longitudinal_stiffness * slip_ratio / grip;
            const Real ky = t.cornering_stiffness * lateral_slip / grip;
            const Real s = std::hypot(kx, ky); // size of the combined slip
            if (s == 0)
                return {};                               // no slip, no force (and no 0 / 0 below)
            const Real force = grip * curve_shape(t, s); // one curve lookup for the combined slip
            return {force * kx / s, force * ky / s};     // split along the slip direction
        }
        }
        return {}; // not reached: every curve returns above
    }
}
