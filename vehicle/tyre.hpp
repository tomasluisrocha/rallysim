#pragma once
#include <algorithm>
#include <cmath>
#include "core/vec3.hpp"

namespace rallysim
{
    // One tyre (the tyre model ladder). Rungs 1 to 3: linear in slip, capped by load-sensitive
    // friction, with combined slip shared on a friction circle (the linear curve). Two more curves have a peak
    // and a fall-off: peak_slide (rung 4, a brush/Fiala-style curve with one shape for both directions) and tmeasy
    // (rung 5, its own curve for each direction, given at two loads). The rest of the car only calls tyre_forces()
    // and the slip cap, whichever curve is chosen.

    // Shape of the force against slip.
    enum class tyre_curve
    {
        linear,     // straight line of slope C, flat at the grip: no peak, no fall-off (the closed-form tests use it)
        peak_slide, // rises with slope C, peaks smoothly, falls to the sliding force; one shape, both directions
                    // (brush when peak_slip = 3, sliding_ratio = 1)
        tmeasy      // the same kind of curve, but each direction has its own, given at two loads (TMeasy)
    };

    // TMeasy (Rill's TMeasy model): the pure-slip curve of one direction at one load, as numbers
    // that can be read off a measured curve. The peak force is not here: it is the tyre's grip, mu(Fz) Fz (see
    // tmeasy_at).
    struct tmeasy_point
    {
        Real stiffness;     // initial slope: N per unit of slip ratio (along), N per rad (across)
        Real peak_slip;     // where the force peaks: slip ratio (along), tangent of the slip angle (across)
        Real sliding_slip;  // from where the tyre slides fully; past it the force stays at the sliding force
        Real sliding_ratio; // sliding force over peak force (0 to 1)
    };

    // One direction of a TMeasy tyre: its curve at the nominal load Fz0 and at twice it. Between and beyond those
    // two loads the numbers are interpolated (tmeasy_at), as TMeasy does, so one table covers every load.
    struct tmeasy_direction
    {
        tmeasy_point nominal;     // at nominal_load
        tmeasy_point double_load; // at 2 nominal_load
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

        tyre_curve curve = tyre_curve::linear;

        // The peak_slide curve. Its slips are normalised: slip 1 is where the linear tyre would reach the grip,
        // C s = mu(Fz) Fz, so the peak moves to larger slips as the load grows (as a brush tyre's does).
        Real peak_slip = 2.0;     // normalised slip at the peak force; between 1.5 and 3. Above 3 the rise overshoots
                                  // the grip; below 1.5 it bulges above the linear tyre, so braking while cornering
                                  // would add side force (the gate test checks the range)
        Real sliding_slip = 4.0;  // normalised slip from which the tyre slides fully; above peak_slip
        Real sliding_ratio = 0.8; // sliding force over peak force (0 to 1); 1 gives no fall-off

        // The tmeasy curve, one table per direction (the stiffnesses above are not used by it). The numbers are
        // placeholders shaped after published curves of a car tyre (peaks near 9 degrees and 9-10%
        // slip, a slow fall to about 75% when locked), not fitted to a real tyre. Along: peak at 9-9.5% slip,
        // sliding from 80%, 75% left. Across: peak at about 9.5-10 degrees, sliding from 35 degrees, 85% left.
        // The stiffnesses grow with load, so the peak slip angle stays nearly the same at every load.
        tmeasy_direction tmeasy_x{{90000, 0.090, 0.8, 0.75}, {160000, 0.095, 0.8, 0.75}};
        tmeasy_direction tmeasy_y{{30000, 0.165, 0.7, 0.85}, {55000, 0.175, 0.7, 0.85}};
    };

    // Force of one tyre on the car, in the wheel frame (N): x along the wheel, y to its left.
    struct tyre_force
    {
        Real fx = 0, fy = 0;
    };

    // Peak friction coefficient at vertical load fz (N): mu0 (1 - k (fz - fz0) / fz0), never below
    // mu0 (1 + k) / 2. That is its value where the grip mu(fz) fz peaks, fz = fz0 (1 + k) / (2 k), so the grip
    // never falls as the load rises: it is continuous there and linear in the load after.
    // Shared with the single-track model, so both use the same arithmetic.
    inline Real load_sensitive_friction(Real mu0, Real sensitivity, Real nominal_load, Real fz)
    {
        const Real drop = sensitivity * (fz - nominal_load) / nominal_load;
        return mu0 * std::max(static_cast<Real>(1) - drop, (static_cast<Real>(1) + sensitivity) / 2);
    }

    inline Real peak_friction(const tyre_params &t, Real fz)
    {
        return load_sensitive_friction(t.friction_coefficient, t.load_sensitivity, t.nominal_load, fz);
    }

    // Largest force the tyre can take from the road at load fz (N): mu(fz) fz, zero when unloaded. It is the peak of
    // every curve, in every direction.
    inline Real tyre_grip(const tyre_params &t, Real fz)
    {
        return fz > 0 ? peak_friction(t, fz) * fz : static_cast<Real>(0);
    }

    // The shape every peaked curve shares: force over peak force (0 to 1) at a slip s >= 0
    // scaled so the curve starts with slope 1 (s = 1 is where a straight line of the initial slope would reach the
    // peak force). A cubic rises from 0 with slope 1 to 1 at p, a half-cosine falls to r at q, and it stays at r.
    // Both joins are flat, so the slope never jumps. Needs 1.5 <= p <= 3 (above 3 the cubic overshoots 1; below 1.5
    // it bulges above its starting slope, so braking while cornering would add side force) and q > p.
    inline Real peak_slide_shape(Real s, Real p, Real q, Real r)
    {
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

    // The peak_slide curve: force over grip at normalised slip s (the size of (Cx sx, C_alpha sy) / grip; the
    // caller applies the direction), with this tyre's peak_slip, sliding_slip and sliding_ratio.
    inline Real curve_shape(const tyre_params &t, Real s)
    {
        return peak_slide_shape(s, t.peak_slip, t.sliding_slip, t.sliding_ratio);
    }

    // A TMeasy direction at load fz, interpolated as TMeasy does from its two given loads (f = fz / Fz0):
    //   the stiffness, like a force, goes through zero and is quadratic in the load through both given points:
    //     k(f) = f (2 k1 - k2 / 2 - (k1 - k2 / 2) f)      (k1 at f = 1, k2 at f = 2)
    //   the slips and the sliding ratio are linear in the load through both points.
    // The peak force is the grip, mu(fz) fz: TMeasy's peak forces at Fz0 and 2 Fz0, mu0 Fz0 and 2 Fz0 mu0 (1 - k),
    // interpolated the same quadratic way, give exactly the linear load sensitivity the other curves use, so all
    // three curves share one grip. Kept positive and ordered (sliding past the peak) for any load.
    inline tmeasy_point tmeasy_at(const tmeasy_direction &d, Real fz, Real nominal_load)
    {
        const Real f = std::max(fz, static_cast<Real>(0)) / nominal_load;
        const tmeasy_point &a = d.nominal, &b = d.double_load;
        auto linear = [f](Real at1, Real at2) { return at1 + (at2 - at1) * (f - 1); };
        const Real stiffness = f * (2 * a.stiffness - 0.5 * b.stiffness - (a.stiffness - 0.5 * b.stiffness) * f);
        const Real peak = std::max(linear(a.peak_slip, b.peak_slip), static_cast<Real>(1e-4));
        const Real sliding = std::max(linear(a.sliding_slip, b.sliding_slip), 1.01 * peak);
        const Real ratio = std::clamp(linear(a.sliding_ratio, b.sliding_ratio), static_cast<Real>(0), static_cast<Real>(1));
        return {std::max(stiffness, static_cast<Real>(0)), peak, sliding, ratio};
    }

    // The two directions of a tyre.
    enum class slip_axis
    {
        x, // along the wheel: the slip ratio
        y  // across the wheel: the lateral slip (tangent of the slip angle)
    };

    // The pure-slip curve of one direction at load fz, in real slip units, whatever the tyre's curve: how steeply
    // it starts, where it peaks (at the grip), where it slides fully and what is left then. The tests, the slip cap
    // and the tools read the curves through this, so they need not know each model's own numbers.
    //   linear:     straight to the grip at grip / C, which is also where it "slides" (nothing is lost)
    //   peak_slide: peak at peak_slip grip / C, full sliding at sliding_slip grip / C
    //   tmeasy:     the direction's table at this load (tmeasy_at)
    struct slip_curve
    {
        Real stiffness, peak_slip, sliding_slip, sliding_ratio;
    };

    inline slip_curve pure_slip_curve(const tyre_params &t, Real fz, slip_axis axis)
    {
        const Real grip = tyre_grip(t, fz);
        const Real stiffness = axis == slip_axis::x ? t.longitudinal_stiffness : t.cornering_stiffness;
        switch (t.curve)
        {
        case tyre_curve::linear:
            return {stiffness, grip / stiffness, grip / stiffness, 1};
        case tyre_curve::peak_slide:
            return {stiffness, t.peak_slip * grip / stiffness, t.sliding_slip * grip / stiffness, t.sliding_ratio};
        case tyre_curve::tmeasy:
        {
            const tmeasy_point p = tmeasy_at(axis == slip_axis::x ? t.tmeasy_x : t.tmeasy_y, fz, t.nominal_load);
            return {p.stiffness, p.peak_slip, p.sliding_slip, p.sliding_ratio};
        }
        }
        return {stiffness, grip / stiffness, grip / stiffness, 1}; // not reached: every curve returns above
    }

    // TMeasy's combined slip (Rill): each slip is divided by its own direction's peak slip, so 1 is the peak in
    // every direction, and the size of that vector, sigma, is looked up once on a curve whose numbers are blended
    // between the two directions by the slip's direction (cos and sin of its angle):
    //   initial slope (per unit sigma)  |(kx sMx cos, ky sMy sin)|
    //   full sliding (in sigma)          |(sSx / sMx cos, sSy / sMy sin)|
    //   sliding ratio                    |(rx cos, ry sin)|
    // and the peak force is the grip in every direction. A pure slip gives back that direction's own curve. The
    // usual combined-slip method (peak_slide's) divides by the stiffness instead, which forces both
    // directions onto one shape; dividing by the peak slip lets each keep its own. (Rill also weights the two
    // directions in the normalisation; this keeps the plain peak-slip form.)
    //
    // The force is split along (kx sx, ky sy), the direction the linear tyre's force would take, not along the
    // normalised slip. Split along the normalised slip, the force at small slip is |(kx sMx, ky sMy)|-blended times
    // sigma, so where the two directions' kx sMx and ky sMy differ (8100 and 4950 N here) a little slip ratio turns
    // the blend and raises the side force: braking gently in a corner would add side grip (the gate test caught
    // it). Split along (kx sx, ky sy), each direction is exactly linear at small slip, and the side force is
    // ky sy shape(s) / s, which never rises with the slip ratio because shape(s) / s never rises (p >= 1.5).
    //
    // On peak_slide_shape's scale (slope 1 at the start) the peak is at p = slope / grip; p is held within 1.5 to 3,
    // where that shape is valid, by adjusting the starting slope, so the peak stays exactly at sigma = 1. The
    // default tables stay inside that range from 500 N to 10 kN (the gate test checks), so it only guards a table
    // pushed to extreme loads.
    struct tmeasy_slip
    {
        Real sigma;   // size of the peak-normalised combined slip (1 = at the peak)
        Real ux, uy;  // direction to split the force along: (kx sx, ky sy), made unit
        Real p, q, r; // the blended curve on peak_slide_shape's scale: peak, full sliding, sliding ratio
    };

    inline tmeasy_slip tmeasy_combine(const tyre_params &t, Real slip_ratio, Real lateral_slip, Real fz)
    {
        const slip_curve ax = pure_slip_curve(t, fz, slip_axis::x), ay = pure_slip_curve(t, fz, slip_axis::y);
        const Real grip = tyre_grip(t, fz);
        const Real nx = slip_ratio / ax.peak_slip, ny = lateral_slip / ay.peak_slip;
        const Real sigma = std::hypot(nx, ny);
        const Real cx = sigma > 0 ? nx / sigma : 1, cy = sigma > 0 ? ny / sigma : 0;
        const Real slope = std::hypot(ax.stiffness * ax.peak_slip * cx, ay.stiffness * ay.peak_slip * cy);
        const Real p = grip > 0 ? std::clamp(slope / grip, static_cast<Real>(1.5), static_cast<Real>(3)) : static_cast<Real>(2);
        const Real sliding = std::hypot(ax.sliding_slip / ax.peak_slip * cx, ay.sliding_slip / ay.peak_slip * cy);
        const Real ratio = std::hypot(ax.sliding_ratio * cx, ay.sliding_ratio * cy);
        const Real lx = ax.stiffness * slip_ratio, ly = ay.stiffness * lateral_slip, l = std::hypot(lx, ly);
        const Real ux = l > 0 ? lx / l : cx, uy = l > 0 ? ly / l : cy;
        return {sigma, ux, uy, p, sliding * p, ratio};
    }

    // Largest normalised slip the relaxed slips of a linear or peak_slide tyre may build up to, at rolling speed V
    // (m/s), on the scale where 1 is where the linear tyre reaches the grip (slip_cap_scale explains the cap).
    // The linear force is flat from 1. The peak_slide force changes up to sliding_slip, so a rolling tyre is
    // capped there, and blends down to peak_slip at rest. A tmeasy tyre's cap is on another scale and depends on
    // the slip's direction, so slip_cap_scale works it out itself and this is not used for it.
    inline Real max_normalised_slip(const tyre_params &t, Real rolling_speed)
    {
        switch (t.curve)
        {
        case tyre_curve::linear:
        case tyre_curve::tmeasy:
            return 1; // the linear force reaches the grip at s = 1
        case tyre_curve::peak_slide:
        {
            const Real rolling = std::clamp(rolling_speed / t.damping_fade_speed, static_cast<Real>(0), static_cast<Real>(1));
            return t.peak_slip + (t.sliding_slip - t.peak_slip) * rolling; // past full sliding the curve is flat
        }
        }
        return 1; // not reached: every curve returns above
    }

    // How far the relaxed slips (the tyre's twist, sigma times the slip) may go: the factor, at most 1, to scale
    // both slips by (cap_slips in wheel.hpp). A tyre cannot hold more twist than its force needs. While it rolls the
    // cap is where the force stops changing (full sliding; for the linear tyre, where it reaches the grip), so it
    // can slide down the fall-off. But the twist is released when the car stops, and a stopped tyre holds no more
    // than its peak force (static grip holds up to the peak), so the cap blends down to the peak as the rolling
    // speed V (m/s) falls below damping_fade_speed, the same fade as the low-speed damping. Zero when unloaded: a
    // tyre in the air is not twisted.
    inline Real slip_cap_scale(const tyre_params &t, Real slip_ratio, Real lateral_slip, Real fz, Real V)
    {
        const Real grip = tyre_grip(t, fz);
        if (grip <= 0)
            return 0;
        switch (t.curve)
        {
        case tyre_curve::linear:
        case tyre_curve::peak_slide:
        {
            // on the scale where 1 is where the linear tyre reaches the grip
            const Real size = std::hypot(t.longitudinal_stiffness * slip_ratio, t.cornering_stiffness * lateral_slip);
            const Real limit = max_normalised_slip(t, V) * grip;
            return size > limit ? limit / size : 1;
        }
        case tyre_curve::tmeasy:
        {
            // on the peak-normalised scale (1 = the peak), in this slip's direction
            const tmeasy_slip c = tmeasy_combine(t, slip_ratio, lateral_slip, fz);
            const Real rolling = std::clamp(V / t.damping_fade_speed, static_cast<Real>(0), static_cast<Real>(1));
            const Real limit = 1 + (c.q / c.p - 1) * rolling;
            return c.sigma > limit ? limit / c.sigma : 1;
        }
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
    // peak_slide: each slip is normalised by its own stiffness (1 is where the linear tyre
    // would reach the grip), the size of the combined slip is looked up once on curve_shape, and the force is
    // split along the direction of the slip. So the two directions share one grip, a locked or spinning wheel
    // loses its side force, and at small slip the force is the linear tyre's.
    //
    // tmeasy: each slip is normalised by its own peak slip, the curve's numbers are blended by the slip's direction
    // (tmeasy_combine), the size sigma is looked up once, and the force is split along (kx sx, ky sy), the linear
    // tyre's direction, so each direction is exactly linear at small slip.
    //
    // All: positive slip gives positive force, which reduces the slip; never more than the grip.
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
            // Normalise each slip by its stiffness: 1 is where the linear tyre would reach the grip.
            const Real kx = t.longitudinal_stiffness * slip_ratio / grip;
            const Real ky = t.cornering_stiffness * lateral_slip / grip;
            const Real s = std::hypot(kx, ky); // size of the combined slip
            if (s == 0)
                return {};                               // no slip, no force (and no 0 / 0 below)
            const Real force = grip * curve_shape(t, s); // one curve lookup for the combined slip
            return {force * kx / s, force * ky / s};     // split along the slip direction
        }
        case tyre_curve::tmeasy:
        {
            const tmeasy_slip c = tmeasy_combine(t, slip_ratio, lateral_slip, fz);
            if (c.sigma == 0)
                return {}; // no slip, no force
            // sigma = 1 is the peak; on peak_slide_shape's scale that is p
            const Real force = grip * peak_slide_shape(c.sigma * c.p, c.p, c.q, c.r);
            return {force * c.ux, force * c.uy}; // split along (kx sx, ky sy)
        }
        }
        return {}; // not reached: every curve returns above
    }
}
