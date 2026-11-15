#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "runner/builtin/tyre_rig.hpp"
#include "tests/check.hpp"
#include "vehicle/tyre.hpp"

using namespace rallysim;

// The tyre gate: before a tyre curve goes on the car, its force curves must show zero force at
// zero slip, force that opposes slip, force bounded by mu Fz, the right symmetry, and a recognisable peak and
// fall-off. The gate also asks for a smooth curve (no jump in slope after the peak) and zero force at zero load.
//
// Every curve of the tyre rig (runner/builtin/tyre_rig.hpp) is checked, so the curves tested are the ones plotted by
// tools/tyre_plot.py: linear (no peak, so the peak checks are replaced by "flat at the grip"), brush, peak_slide and
// tmeasy. The checks read each curve's peak, sliding point and stiffness through pure_slip_curve (tyre.hpp), so the
// same checks fit every model.

namespace
{
    const Real kLoads[] = {1500, 3000, 6000};

    // A grid of slips: slip ratio -1 to 1 and slip angle -30 to 30 degrees (as its tangent, the lateral slip).
    std::vector<Real> slip_ratios()
    {
        std::vector<Real> v;
        for (int i = -20; i <= 20; ++i)
            v.push_back(i * 0.05);
        return v;
    }

    std::vector<Real> lateral_slips()
    {
        std::vector<Real> v;
        for (int i = -20; i <= 20; ++i)
            v.push_back(std::tan(i * 1.5 * kPi / 180));
        return v;
    }

    std::string label(const std::string &curve, const char *what)
    {
        return curve + ": " + what;
    }

    void check_gate(const std::string &name, const tyre_params &t, bool expect_peak)
    {
        const char *n = name.c_str();

        // 1. the shape parameters are in the ranges tyre_params documents. For the cubic s + a s^2 + b s^3, the force
        // over the slip, f(s) / s = 1 + a s + b s^2, must never rise, or a wheel that brakes while cornering gains
        // side force (check 11). Its slope a + 2 b s runs from a = (3 - 2p) / p^2 at s = 0 to -1 / p^2 at the peak,
        // so it never rises when a <= 0, that is p >= 1.5. Above p = 3 the cubic overshoots the grip.
        if (t.curve == tyre_curve::peak_slide)
        {
            check_true(label(name, "1.5 <= peak_slip <= 3").c_str(), t.peak_slip >= 1.5 && t.peak_slip <= 3);
            check_true(label(name, "sliding_slip > peak_slip").c_str(), t.sliding_slip > t.peak_slip);
            check_true(label(name, "0 <= sliding_ratio <= 1").c_str(), t.sliding_ratio >= 0 && t.sliding_ratio <= 1);
        }
        // A tmeasy tyre uses the same shape, with p = stiffness x peak slip / grip worked out for each direction at
        // each load. The tables must keep p in 1.5..3 at every load the car can see, in both directions, or
        // tmeasy_combine has to bend the starting slope to keep the shape valid.
        if (t.curve == tyre_curve::tmeasy)
        {
            bool in_range = true, ordered = true;
            Real lowest = 10, highest = 0;
            for (Real fz = 500; fz <= 10000; fz += 250)
                for (slip_axis axis : {slip_axis::x, slip_axis::y})
                {
                    const slip_curve c = pure_slip_curve(t, fz, axis);
                    const Real p = c.stiffness * c.peak_slip / tyre_grip(t, fz);
                    lowest = std::min(lowest, p);
                    highest = std::max(highest, p);
                    in_range = in_range && p >= 1.5 && p <= 3;
                    ordered = ordered && c.sliding_slip > c.peak_slip && c.sliding_ratio >= 0 && c.sliding_ratio <= 1;
                }
            std::printf("%-11s shape p from 500 N to 10 kN, both directions: %.3f to %.3f\n", n, lowest, highest);
            check_true(label(name, "1.5 <= p <= 3 at every load from 500 N to 10 kN, both directions").c_str(), in_range);
            check_true(label(name, "sliding past the peak, sliding ratio 0..1, at every load").c_str(), ordered);
        }

        // 2. zero slip gives exactly zero force, at any load
        bool zero_slip = true;
        for (Real fz : {0.0, 1500.0, 3000.0, 6000.0})
        {
            const tyre_force f = tyre_forces(t, 0, 0, fz);
            zero_slip = zero_slip && f.fx == 0 && f.fy == 0;
        }
        check_true(label(name, "zero slip gives exactly zero force").c_str(), zero_slip);

        // 3. zero (or negative) load gives exactly zero force, at any slip
        bool zero_load = true;
        for (Real fz : {0.0, -100.0})
            for (Real sx : slip_ratios())
                for (Real sy : {-0.3, 0.0, 0.2})
                {
                    const tyre_force f = tyre_forces(t, sx, sy, fz);
                    zero_load = zero_load && f.fx == 0 && f.fy == 0;
                }
        check_true(label(name, "zero load gives exactly zero force").c_str(), zero_load);

        // 4-6. over the whole grid of slips and loads: symmetric, opposing the slip, and within the grip
        bool symmetric = true, opposes = true, bounded = true;
        Real worst_use = 0;
        for (Real fz : kLoads)
        {
            const Real grip = tyre_grip(t, fz);
            for (Real sx : slip_ratios())
                for (Real sy : lateral_slips())
                {
                    const tyre_force f = tyre_forces(t, sx, sy, fz);
                    const tyre_force mx = tyre_forces(t, -sx, sy, fz); // braking instead of driving
                    const tyre_force my = tyre_forces(t, sx, -sy, fz); // turning the other way
                    symmetric = symmetric && std::fabs(mx.fx + f.fx) <= 1e-9 && std::fabs(mx.fy - f.fy) <= 1e-9 &&
                                std::fabs(my.fx - f.fx) <= 1e-9 && std::fabs(my.fy + f.fy) <= 1e-9;
                    opposes = opposes && f.fx * sx >= 0 && f.fy * sy >= 0;
                    const Real use = std::hypot(f.fx, f.fy) / grip;
                    worst_use = std::max(worst_use, use);
                    bounded = bounded && use <= 1 + 1e-12;
                }
        }
        std::printf("%-11s largest force over grip on the grid: %.12f\n", n, worst_use);
        check_true(label(name, "symmetric: driving/braking and left/right mirror each other").c_str(), symmetric);
        check_true(label(name, "force has the sign of its slip (it reduces the slip)").c_str(), opposes);
        check_true(label(name, "force never above mu(Fz) Fz").c_str(), bounded);

        // 7. at small slip the force is the curve's starting stiffness times the slip, in each direction and at each
        // load (constant for linear and peak_slide; for tmeasy it grows with the load)
        for (Real fz : kLoads)
        {
            const Real tiny = 1e-6;
            const tyre_force fx = tyre_forces(t, tiny, 0, fz), fy = tyre_forces(t, 0, tiny, fz);
            const slip_curve cx = pure_slip_curve(t, fz, slip_axis::x), cy = pure_slip_curve(t, fz, slip_axis::y);
            char what[200];
            std::snprintf(what, sizeof what, "%s: Fz %.0f: initial slope along the wheel = its stiffness", n, fz);
            check_near(what, fx.fx / tiny / cx.stiffness, 1.0, 1e-4);
            std::snprintf(what, sizeof what, "%s: Fz %.0f: initial slope across the wheel = its stiffness", n, fz);
            check_near(what, fy.fy / tiny / cy.stiffness, 1.0, 1e-4);
            // and with both slips at once, each direction still linear in its own slip: a little braking neither
            // adds to nor takes from the side force while the tyre is far from its limit
            const tyre_force both = tyre_forces(t, tiny, 2 * tiny, fz);
            std::snprintf(what, sizeof what, "%s: Fz %.0f: small combined slip, each direction linear in its own slip", n, fz);
            check_true(what, std::fabs(both.fx / (tiny * cx.stiffness) - 1) < 1e-4 &&
                                 std::fabs(both.fy / (2 * tiny * cy.stiffness) - 1) < 1e-4);
        }

        // 8-9. along the wheel at each load: the force reaches the grip exactly at the peak slip, rising all the way
        // there; past it, a curve with a fall-off stays below the grip; fully sliding, what is left is
        // sliding_ratio x grip. (A sweep alone cannot find the peak to 1e-9: the steps miss the exact peak slip, and
        // the brush curve is so flat at its peak that it is within 1e-9 of the grip well before it.)
        for (Real fz : kLoads)
        {
            const Real grip = tyre_grip(t, fz);
            const slip_curve curve = pure_slip_curve(t, fz, slip_axis::x);
            const Real peak_at = curve.peak_slip; // the linear tyre "peaks" where it reaches the grip
            const bool falls = expect_peak && curve.sliding_ratio < 1;
            const Real step = 1e-5;
            bool rising = true, below_after = true;
            Real previous = 0;
            for (int i = 1; i <= 100000; ++i) // slip ratio 0 to 1
            {
                const Real sx = i * step, f = tyre_forces(t, sx, 0, fz).fx;
                if (sx < peak_at)
                    rising = rising && f > previous && f < grip;
                else if (falls && sx > peak_at + step)
                    below_after = below_after && f < grip;
                previous = f;
            }
            const Real at_peak = tyre_forces(t, peak_at, 0, fz).fx;
            const Real locked = tyre_forces(t, 1.0, 0, fz).fx; // a locked or freely spinning wheel (past full sliding)
            const Real ratio = curve.sliding_ratio;
            std::printf("%-11s Fz %4.0f N: grip %7.1f N, at the peak slip ratio %.5f: %7.1f N; at slip ratio 1: %7.1f N\n",
                        n, fz, grip, peak_at, at_peak, locked);

            char what[200];
            std::snprintf(what, sizeof what, "%s: Fz %.0f: the force is exactly the grip at the peak slip", n, fz);
            check_near(what, at_peak / grip, 1.0, 1e-12);
            std::snprintf(what, sizeof what, "%s: Fz %.0f: rising, and below the grip, all the way to the peak", n, fz);
            check_true(what, rising);
            if (falls)
            {
                std::snprintf(what, sizeof what, "%s: Fz %.0f: below the grip past the peak (it falls off)", n, fz);
                check_true(what, below_after);
            }
            std::snprintf(what, sizeof what, "%s: Fz %.0f: fully sliding, the force is sliding_ratio x grip", n, fz);
            check_near(what, locked / grip, ratio, 1e-12);
        }

        // 10. smooth: no jump in slope at the peak or at full sliding, in each direction. The slope just left and
        // just right of each join, as a fraction of the starting stiffness, is near zero on both sides (both joins
        // are flat), and the force has no jump.
        if (expect_peak)
        {
            const Real fz = 3000;
            for (slip_axis axis : {slip_axis::x, slip_axis::y})
            {
                const slip_curve c = pure_slip_curve(t, fz, axis);
                auto force = [&](Real s)
                { return axis == slip_axis::x ? tyre_forces(t, s, 0, fz).fx : tyre_forces(t, 0, s, fz).fy; };
                for (Real s : {c.peak_slip, c.sliding_slip})
                {
                    const Real h = 1e-6 * s;
                    const Real left = (force(s) - force(s - h)) / h / c.stiffness;
                    const Real right = (force(s + h) - force(s)) / h / c.stiffness;
                    char what[200];
                    std::snprintf(what, sizeof what, "%s: %s: flat on both sides of slip %g (no kink)", n,
                                  axis == slip_axis::x ? "along" : "across", s);
                    check_true(what, std::fabs(left) < 1e-4 && std::fabs(right) < 1e-4);
                    std::snprintf(what, sizeof what, "%s: %s: no jump in force at slip %g", n,
                                  axis == slip_axis::x ? "along" : "across", s);
                    check_near(what, force(s + h), force(s - h), 1e-4 * tyre_grip(t, fz));
                }
            }
        }
        else
        {
            // the linear tyre has no peak: straight up to the grip, then flat
            const Real fz = 3000, grip = tyre_grip(t, fz);
            check_near(label(name, "flat at the grip past s = 1").c_str(),
                       tyre_forces(t, 2 * grip / t.longitudinal_stiffness, 0, fz).fx, grip, 1e-9);
        }

        // 11. combined slip: the two directions share one grip
        {
            const Real fz = 3000, grip = tyre_grip(t, fz);
            bool pure_matches = true, side_falls = true;
            for (Real angle : {2.0, 4.0, 8.0})
            {
                const Real sy = std::tan(angle * kPi / 180);
                // with no slip ratio, the same as pure lateral slip
                pure_matches = pure_matches && tyre_forces(t, 0, sy, fz).fx == 0;
                // the side force never grows as the wheel slips along (brakes or spins up)
                Real previous = std::fabs(tyre_forces(t, 0, sy, fz).fy);
                for (int i = 1; i <= 1000; ++i)
                {
                    const Real fy = std::fabs(tyre_forces(t, i * 0.001, sy, fz).fy);
                    side_falls = side_falls && fy <= previous + 1e-9;
                    previous = fy;
                }
            }
            check_true(label(name, "combined: no slip ratio gives a pure side force").c_str(), pure_matches);
            check_true(label(name, "combined: the side force never grows as the slip ratio grows").c_str(), side_falls);
            if (expect_peak)
            {
                // a locked wheel at a 4 degree slip angle keeps under 5% of its grip across the wheel
                const tyre_force f = tyre_forces(t, -1.0, std::tan(4 * kPi / 180), fz);
                std::printf("%-11s locked at 4 deg: Fx %.1f N, Fy %.1f N (grip %.1f N)\n", n, f.fx, f.fy, grip);
                check_true(label(name, "combined: a locked wheel loses its side force").c_str(), std::fabs(f.fy) < 0.05 * grip);
            }
        }

        // 13. where the side force peaks, measured from the forces, at a light and a heavy load. Real tyres peak at
        // nearly the same slip angle at any load, because their cornering stiffness grows with the load as their
        // grip does; tmeasy's two-load tables do that. peak_slide and linear keep one stiffness, so their peak
        // moves out in proportion to the grip (printed, not checked).
        if (expect_peak)
        {
            auto peak_angle = [&](Real fz)
            {
                Real best = 0, at = 0;
                for (int i = 1; i <= 4000; ++i) // 0.01 to 40 degrees
                {
                    const Real deg = i * 0.01, f = tyre_forces(t, 0, std::tan(deg * kPi / 180), fz).fy;
                    if (f > best)
                    {
                        best = f;
                        at = deg;
                    }
                }
                return at;
            };
            const Real light = peak_angle(1500), heavy = peak_angle(6000);
            std::printf("%-11s side force peaks at %.2f deg at 1500 N, %.2f deg at 6000 N\n", n, light, heavy);
            if (t.curve == tyre_curve::tmeasy)
                check_true(label(name, "the peak slip angle is nearly the same at 1500 N and 6000 N (within 15%)").c_str(),
                           std::fabs(heavy / light - 1) < 0.15);
        }
    }

    // The tmeasy stiffness interpolated with no hold past its peak: the old law, kept to compare against below it.
    tmeasy_point unheld_tmeasy_at(const tmeasy_direction &d, Real fz, Real nominal_load)
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

    // The tmeasy force from given points of each direction: the combination and the split along (kx sx, ky sy).
    tyre_force tmeasy_force_from(const tmeasy_point &ax, const tmeasy_point &ay, Real grip, Real sx, Real sy)
    {
        const Real nx = sx / ax.peak_slip, ny = sy / ay.peak_slip, sigma = std::hypot(nx, ny);
        const Real cx = nx / sigma, cy = ny / sigma;
        const Real slope = std::hypot(ax.stiffness * ax.peak_slip * cx, ay.stiffness * ay.peak_slip * cy);
        const Real p = std::clamp(slope / grip, static_cast<Real>(1.5), static_cast<Real>(3));
        const Real q = std::hypot(ax.sliding_slip / ax.peak_slip * cx, ay.sliding_slip / ay.peak_slip * cy) * p;
        const Real r = std::hypot(ax.sliding_ratio * cx, ay.sliding_ratio * cy);
        const Real lx = ax.stiffness * sx, ly = ay.stiffness * sy, l = std::hypot(lx, ly);
        const Real force = grip * peak_slide_shape(sigma * p, p, q, r);
        const Real ux = lx / l, uy = ly / l;
        return {force * ux, force * uy};
    }

    // The tmeasy stiffness k(f) = f (A - B f) peaks at f = A / (2 B) (5 nominal loads along, 6.5 across with the
    // default tables) and would fall to zero at A / B; past the peak it stays there. A stiffness falling in one
    // direction only turns the force sideways, since the force is split along (kx sx, ky sy).
    void check_tmeasy_stiffness_hold()
    {
        tyre_params t;
        t.curve = tyre_curve::tmeasy;
        const Real fz0 = t.nominal_load;
        auto k = [&](Real n, slip_axis axis) { return pure_slip_curve(t, n * fz0, axis).stiffness; };

        // by hand: along A 100000, B 10000, peak 5 x 50000; across A 32500, B 2500, peak 6.5 x 16250
        const Real along_at[] = {4, 5, 6, 8, 10, 15}, along_k[] = {240000, 250000, 250000, 250000, 250000, 250000};
        for (int i = 0; i < 6; ++i)
        {
            char what[160];
            std::snprintf(what, sizeof what, "tmeasy hold: along stiffness at %g nominal loads", along_at[i]);
            check_near(what, k(along_at[i], slip_axis::x), along_k[i], 1e-6);
        }
        const Real across_at[] = {4, 6.5, 8, 11, 15}, across_k[] = {90000, 105625, 105625, 105625, 105625};
        for (int i = 0; i < 5; ++i)
        {
            char what[160];
            std::snprintf(what, sizeof what, "tmeasy hold: across stiffness at %g nominal loads", across_at[i]);
            check_near(what, k(across_at[i], slip_axis::y), across_k[i], 1e-6);
        }

        // the stiffness never falls with load; the curve's starting slope is the stiffness held within the
        // shape's bounds, 1.5 to 3 grip / peak slip, and never falls from 6 nominal loads up
        bool stiffness_rises = true, slope_rises = true;
        Real worst_slope = 0, last_k[2] = {0, 0}, last_slope[2] = {0, 0};
        for (int i = 1; i <= 30; ++i)
        {
            const Real n = 0.5 * i, fz = n * fz0, grip = tyre_grip(t, fz), h = 1e-7;
            for (int a = 0; a < 2; ++a)
            {
                const slip_axis axis = a == 0 ? slip_axis::x : slip_axis::y;
                const slip_curve c = pure_slip_curve(t, fz, axis);
                const Real slope = (a == 0 ? tyre_forces(t, h, 0, fz).fx : tyre_forces(t, 0, h, fz).fy) / h;
                const Real expected = std::clamp(c.stiffness, 1.5 * grip / c.peak_slip, 3 * grip / c.peak_slip);
                worst_slope = std::max(worst_slope, std::fabs(slope / expected - 1));
                stiffness_rises = stiffness_rises && c.stiffness >= last_k[a];
                if (n > 6)
                    slope_rises = slope_rises && slope >= last_slope[a];
                last_k[a] = c.stiffness;
                last_slope[a] = slope;
            }
        }
        std::printf("tmeasy hold: largest relative difference of the starting slope from the bounded stiffness %.3g\n", worst_slope);
        check_true("tmeasy hold: the stiffness never falls with load, 0.5 to 15 nominal loads, both directions", stiffness_rises);
        check_near("tmeasy hold: starting slope = stiffness within 1.5..3 grip / peak slip, 0.5 to 15 nominal loads",
                   worst_slope, 0.0, 1e-5);
        check_true("tmeasy hold: the starting slope never falls from 6 nominal loads up, both directions", slope_rises);

        // braking with a little lateral slip: the force points along (kx sx, ky sy), with the held stiffnesses
        const Real sx = -0.05, sy = 0.01;
        for (Real n : {8.0, 11.0})
        {
            const tyre_force f = tyre_forces(t, sx, sy, n * fz0);
            std::printf("tmeasy hold: %g nominal loads, slip ratio %g, lateral slip %g: fx %.1f N, fy %.1f N\n", n, sx, sy,
                        f.fx, f.fy);
            char what[160];
            std::snprintf(what, sizeof what, "tmeasy hold: %g nominal loads: fy / fx = ky sy / (kx sx) = -0.0845", n);
            check_near(what, f.fy / f.fx / (105625 * sy / (250000 * sx)), 1.0, 1e-9);
        }

        // below the peak, the same bits as with no hold
        for (Real n : {2.0, 4.0})
        {
            const Real fz = n * fz0;
            const tmeasy_point ax = unheld_tmeasy_at(t.tmeasy_x, fz, fz0), ay = unheld_tmeasy_at(t.tmeasy_y, fz, fz0);
            const tyre_force want = tmeasy_force_from(ax, ay, tyre_grip(t, fz), sx, sy), got = tyre_forces(t, sx, sy, fz);
            const slip_curve cx = pure_slip_curve(t, fz, slip_axis::x), cy = pure_slip_curve(t, fz, slip_axis::y);
            char what[160];
            std::snprintf(what, sizeof what, "tmeasy hold: %g nominal loads: both tables as with no hold (==)", n);
            auto same = [](const slip_curve &c, const tmeasy_point &p)
            {
                return c.stiffness == p.stiffness && c.peak_slip == p.peak_slip && c.sliding_slip == p.sliding_slip &&
                       c.sliding_ratio == p.sliding_ratio;
            };
            check_true(what, same(cx, ax) && same(cy, ay));
            std::snprintf(what, sizeof what, "tmeasy hold: %g nominal loads: the force as with no hold (==)", n);
            check_true(what, got.fx == want.fx && got.fy == want.fy);
        }
    }
}

int main()
{
    for (const auto &[name, t] : tyre_rig_curves())
        check_gate(name, t, t.curve != tyre_curve::linear);
    check_tmeasy_stiffness_hold();

    // 12. the brush model: with its peak at 3 and no fall-off, the cubic is exactly
    // s - s^2 / 3 + s^3 / 27 up to s = 3, then 1
    {
        tyre_params brush;
        brush.curve = tyre_curve::peak_slide;
        brush.peak_slip = 3;
        brush.sliding_ratio = 1;
        Real worst = 0;
        for (int i = 0; i <= 600; ++i)
        {
            const Real s = i * 0.01;
            const Real formula = s <= 3 ? s - s * s / 3 + s * s * s / 27 : 1.0;
            worst = std::max(worst, std::fabs(curve_shape(brush, s) - formula));
        }
        std::printf("brush: largest difference from the brush formula %.3g\n", worst);
        check_near("brush: curve_shape with peak_slip 3 and no fall-off is the brush formula", worst, 0.0, 1e-12);
    }

    return finish("tyre_gate");
}
