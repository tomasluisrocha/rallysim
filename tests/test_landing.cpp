#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include "tests/check.hpp"
#include "vehicle/four_wheel.hpp"
#include "vehicle/four_wheel_energy.hpp"

using namespace rallysim;

// Landing: the default car at rest, lifted above its ride height and let go, with no inputs. It falls freely until
// its springs reach the ground, lands on them and settles. Checked: the fall against its closed form, that the
// landing never gains energy, that the tyre loads stay between what the dampers and springs can give, that the car
// ends at rest at its ride height, and that the energy residual halves with dt.

namespace
{
    struct landing
    {
        bool fall_ok = true;        // every free-fall step matched the closed form
        long fall_steps = 0;        // steps with every corner off the ground
        Real worst_fall = 0;        // largest free-fall error (m)
        bool never_gained = true;   // stored energy never above its starting value
        Real peak[2] = {0, 0};      // largest tyre load, front and rear (N)
        Real x_max[2] = {0, 0};     // largest spring compression, front and rear (m)
        Real lowest = 1e9;          // lowest CG (m)
        Real residual = 0;          // energy residual at the end (J)
        // unsprung runs only
        Real travel[2] = {0, 0};    // largest strut travel past rest, front and rear (m)
        Real stop_max[2] = {0, 0};  // largest bump stop force, front and rear (N)
        long stop_steps[2] = {0, 0}; // corner-steps with the bump stop engaged
        Real fz_min = 1e9;          // smallest tyre load, any corner (N)
        Real damper_min = 1e9;      // smallest damper rate from rates_of, over the corners (W)
        Real rise_max = 0;          // largest step-to-step rise of the total energy (J)
        Real e_strut = 0, e_tyre = 0, e_droop = 0, e_bump = 0; // peak of each stored element (J)
        four_wheel_state end;
    };

    // Height of the whole car's CG (m). With springs it is the body's; with a wheel mass it is the sprung body and
    // the wheels together, which fall freely while no tyre touches because the struts only push between them.
    Real car_cg_z(const four_wheel_state &s, const four_wheel_params &p)
    {
        if (p.vertical != vertical_model::unsprung)
            return s.body.pos.z;
        const Real ms = p.sprung_mass();
        Real moment = ms * s.body.pos.z;
        for (std::size_t i = 0; i < kWheels; ++i)
            moment += p.corner(i).unsprung_mass * s.corners[i].z;
        return moment / p.chassis.mass;
    }

    landing drop(const four_wheel_params &p, Real lift, Real dt, Real seconds)
    {
        landing l;
        const bool wheels = p.vertical == vertical_model::unsprung;
        four_wheel_state s = four_wheel_initial(p, 0.0);
        s.body.pos.z += lift;
        for (corner_state &c : s.corners)
            c.z += lift;
        const Real z0 = car_cg_z(s, p);
        const four_wheel_input none{};
        energy_books books(s, p);
        const Real e0 = books.start();
        const long steps = std::lround(seconds / dt);
        bool falling = true; // no corner has touched yet
        Real e_prev = e0;
        for (long i = 0; i < steps; ++i)
        {
            const four_wheel_forces f = evaluate(s, p, none);
            const energy_rates rates = rates_of(s, p, f);
            books.add(rates, dt);
            if (wheels)
            {
                l.damper_min = std::min(l.damper_min, rates.damper);
                Real e_strut = 0, e_tyre = 0, e_droop = 0, e_bump = 0;
                for (std::size_t k = 0; k < kWheels; ++k)
                {
                    const corner_params &c = p.corner(k);
                    const corner_load &cl = f.corner[k];
                    const int axle = is_front(k) ? 0 : 1;
                    const Real rest = p.strut_static_deflection(k);
                    l.travel[axle] = std::max(l.travel[axle], cl.deflection - rest);
                    l.stop_max[axle] = std::max(l.stop_max[axle], cl.stop);
                    if (cl.stop > 0)
                        ++l.stop_steps[axle];
                    l.fz_min = std::min(l.fz_min, f.fz[k]);
                    const Real x = std::max(cl.deflection, static_cast<Real>(0));
                    const Real d = std::max(cl.tyre_deflection, static_cast<Real>(0));
                    const Real droop = std::min(cl.deflection - (rest - c.droop), static_cast<Real>(0));
                    e_strut = std::max(e_strut, 0.5 * c.spring_rate * x * x);
                    e_tyre = std::max(e_tyre, 0.5 * c.tyre_rate * d * d);
                    e_droop = std::max(e_droop, 0.5 * c.stop_rate * droop * droop);
                    e_bump = std::max(e_bump, bump_stop_energy(cl.deflection - (rest + c.bump_travel),
                                                               c.bump_stop_rate, c.bump_stop_length));
                }
                l.e_strut = std::max(l.e_strut, e_strut);
                l.e_tyre = std::max(l.e_tyre, e_tyre);
                l.e_droop = std::max(l.e_droop, e_droop);
                l.e_bump = std::max(l.e_bump, e_bump);
            }
            for (std::size_t k = 0; k < kWheels; ++k)
            {
                const int axle = is_front(k) ? 0 : 1;
                l.peak[axle] = std::max(l.peak[axle], f.fz[k]);
                l.x_max[axle] = std::max(l.x_max[axle], f.corner[k].deflection);
                falling = falling && (wheels ? f.corner[k].tyre_deflection : f.corner[k].deflection) <= 0;
            }
            step(s, p, none, dt);
            if (falling)
            {
                // semi-implicit Euler from rest: z = z0 - 0.5 g t^2 - 0.5 g t dt
                const Real t = (i + 1) * dt;
                const Real error = std::fabs(car_cg_z(s, p) - (z0 - 0.5 * kGravity * t * t - 0.5 * kGravity * t * dt));
                l.worst_fall = std::max(l.worst_fall, error);
                l.fall_ok = l.fall_ok && error <= 1e-9;
                ++l.fall_steps;
            }
            const Real e_now = energy_of(s, p).total();
            l.never_gained = l.never_gained && e_now <= e0 * (1 + 1e-12);
            l.rise_max = std::max(l.rise_max, e_now - e_prev);
            e_prev = e_now;
            l.lowest = std::min(l.lowest, s.body.pos.z);
        }
        l.residual = books.residual(energy_of(s, p));
        l.end = s;
        return l;
    }

    // Independent reference: a two-mass quarter-car with its own equations (displacements from rest, + up), RK4.
    struct ref_result
    {
        Real travel = 0; // largest x - x0 (m)
        Real stop = 0;   // largest bump stop force (N)
    };

    Real ref_stop(Real e, Real kb, Real L)
    {
        if (e <= 0)
            return 0;
        return e <= L ? kb * e * e / (2 * L) : kb * (e - L / 2);
    }

    ref_result reference(Real lift)
    {
        const Real ms = 260, mu = 40, k = 50000, c = 3800, kt = 250000, kd = 500000, droop = 0.10, gap = 0.10, L = 0.03,
                   kb = 500000, g = kGravity;
        const Real x0 = ms * g / k, d0 = (ms + mu) * g / kt;
        const Real h = 1e-5;
        const long n = std::lround(1.5 / h);
        ref_result r;
        // y = {zs, zu, vs, vu}
        auto accel = [&](const Real y[4], Real out[4]) {
            const Real x = x0 + y[1] - y[0], xr = y[3] - y[2];
            Real S = k * std::max(x, static_cast<Real>(0)) + c * xr;
            if (x < x0 - droop)
                S += kd * (x - (x0 - droop));
            S += ref_stop(x - x0 - gap, kb, L);
            const Real d = d0 - y[1];
            const Real fz = d > 0 ? kt * d : 0;
            out[0] = y[2];
            out[1] = y[3];
            out[2] = S / ms - g;
            out[3] = (fz - S) / mu - g;
        };
        Real y[4] = {lift, lift, 0, 0};
        for (long i = 0; i < n; ++i)
        {
            const Real x = x0 + y[1] - y[0];
            r.travel = std::max(r.travel, x - x0);
            r.stop = std::max(r.stop, ref_stop(x - x0 - gap, kb, L));
            Real k1[4], k2[4], k3[4], k4[4], t[4];
            accel(y, k1);
            for (int j = 0; j < 4; ++j)
                t[j] = y[j] + 0.5 * h * k1[j];
            accel(t, k2);
            for (int j = 0; j < 4; ++j)
                t[j] = y[j] + 0.5 * h * k2[j];
            accel(t, k3);
            for (int j = 0; j < 4; ++j)
                t[j] = y[j] + h * k3[j];
            accel(t, k4);
            for (int j = 0; j < 4; ++j)
                y[j] += h / 6 * (k1[j] + 2 * k2[j] + 2 * k3[j] + k4[j]);
        }
        return r;
    }

    // Test inputs for the bump stop, not car data: it touches 0.10 m past rest on both axles.
    void set_test_stop(four_wheel_params &p)
    {
        p.front_corner.bump_travel = 0.10;
        p.rear_corner.bump_travel = 0.10;
    }

    // At rest at the ride height, each tyre carrying its share of the weight.
    void check_settled(const char *name, const four_wheel_params &p, const landing &l)
    {
        char what[160];
        std::snprintf(what, sizeof what, "%s: ends at its ride height", name);
        check_near(what, l.end.body.pos.z, p.cg_height, 1e-4);
        const four_wheel_forces f = evaluate(l.end, p, four_wheel_input{});
        for (std::size_t k = 0; k < kWheels; ++k)
        {
            std::snprintf(what, sizeof what, "%s: each tyre back at its static load", name);
            check_near(what, f.fz[k], p.static_load(k), 1.0);
        }
    }
}

int main()
{
    const four_wheel_params p; // springs, 50 kN/m and 3800 N s/m at every corner
    const Real dt = 0.001;

    // 0.3 m
    {
        const Real lift = 0.3;
        const landing l = drop(p, lift, dt, 4.0);
        std::printf("drop 0.3 m: %ld free-fall steps (worst error %.2e m), peak load front %.0f N rear %.0f N, "
                    "deflection front %.3f m rear %.3f m, lowest CG %.3f m, residual %.2f J\n",
                    l.fall_steps, l.worst_fall, l.peak[0], l.peak[1], l.x_max[0], l.x_max[1], l.lowest, l.residual);

        check_true("fall: the car is in the air at first (over 200 steps)", l.fall_steps > 200);
        check_true("fall: z = z0 - 0.5 g t^2 - 0.5 g t dt within 1e-9 until a corner touches", l.fall_ok);
        check_true("landing: the stored energy is never above its starting value", l.never_gained);

        // each corner meets the ground at v = sqrt(2 g fall), the fall being the lift less the spring's static
        // compression; the damper gives c v at once, and the load is never more than c v + k x_max
        const std::size_t corner[2] = {FL, RL};
        const char *const axle_name[2] = {"front", "rear"};
        for (int a = 0; a < 2; ++a)
        {
            const corner_params &c = p.corner(corner[a]);
            const Real v = std::sqrt(2 * kGravity * (lift - p.static_deflection(corner[a])));
            const Real low = 0.9 * c.damper_rate * v, high = c.damper_rate * v + c.spring_rate * l.x_max[a];
            std::printf("landing, %s: touches at %.3f m/s, peak load %.0f N (between %.0f and %.0f N)\n", axle_name[a], v,
                        l.peak[a], low, high);
            char what[160];
            std::snprintf(what, sizeof what, "landing: %s peak load at least 0.9 c v", axle_name[a]);
            check_true(what, l.peak[a] >= low);
            std::snprintf(what, sizeof what, "landing: %s peak load at most c v + k x_max", axle_name[a]);
            check_true(what, l.peak[a] <= high);
        }
        check_settled("drop 0.3 m", p, l);

        // first order, like the step: halving dt halves the residual
        const landing fine = drop(p, lift, 0.5 * dt, 4.0);
        std::printf("convergence: residual %.2f J at 1 ms, %.2f J at 0.5 ms (ratio %.3f)\n", l.residual, fine.residual,
                    fine.residual / l.residual);
        check_true("convergence: halving dt halves the residual (ratio 0.4 to 0.6)",
                   fine.residual / l.residual > 0.4 && fine.residual / l.residual < 0.6);
    }

    // 1 m: a hard landing, still on plain springs (no bump stop yet)
    {
        const landing l = drop(p, 1.0, dt, 4.0);
        std::printf("drop 1 m: peak load front %.0f N rear %.0f N, lowest CG %.3f m, residual %.2f J\n", l.peak[0],
                    l.peak[1], l.lowest, l.residual);
        check_true("drop 1 m: the stored energy is never above its starting value", l.never_gained);
        check_settled("drop 1 m", p, l);
    }

    // The same drops with wheel masses on tyre springs. Both the sprung body and the wheels fall, so the check is on
    // the car's CG; the tyre meets the ground first, so there is no c v bound to compare the peak with.
    {
        four_wheel_params u;
        u.vertical = vertical_model::unsprung;
        const Real lift = 0.3;
        const landing l = drop(u, lift, dt, 4.0);
        std::printf("unsprung drop 0.3 m: %ld free-fall steps (worst error %.2e m), peak tyre load front %.0f N "
                    "rear %.0f N, lowest CG %.3f m, residual %.3f J\n",
                    l.fall_steps, l.worst_fall, l.peak[0], l.peak[1], l.lowest, l.residual);
        check_true("unsprung fall: the car is in the air at first (over 200 steps)", l.fall_steps > 200);
        check_true("unsprung fall: the CG falls as z0 - 0.5 g t^2 - 0.5 g t dt within 1e-9", l.fall_ok);
        check_true("unsprung landing: the stored energy is never above its starting value", l.never_gained);
        for (int a = 0; a < 2; ++a)
            check_true("unsprung landing: the peak tyre load is above the static load", l.peak[a] > u.static_load(a ? RL : FL));
        check_settled("unsprung drop 0.3 m", u, l);

        // the residual is first order, falling at least as fast: halving dt takes it below 0.6 of the coarser one
        const landing fine = drop(u, lift, 0.5 * dt, 4.0);
        std::printf("unsprung convergence: residual %.3f J at 1 ms, %.3f J at 0.5 ms (ratio %.3f)\n", l.residual,
                    fine.residual, fine.residual / l.residual);
        check_true("unsprung convergence: halving dt takes the residual below 0.6 of the coarser one",
                   std::fabs(fine.residual) < 0.6 * std::fabs(l.residual));

        const landing hard = drop(u, 1.0, dt, 4.0);
        std::printf("unsprung drop 1 m: peak tyre load front %.0f N rear %.0f N, lowest CG %.3f m, residual %.3f J\n",
                    hard.peak[0], hard.peak[1], hard.lowest, hard.residual);
        check_true("unsprung drop 1 m: the stored energy is never above its starting value", hard.never_gained);
        check_settled("unsprung drop 1 m", u, hard);

        // C: the placeholder stop stays clear of the default unsprung 1 m drop
        std::printf("unsprung drop 1 m, default stop: travel past rest front %.3f m rear %.3f m, stop never above 0: %s\n",
                    hard.travel[0], hard.travel[1], hard.stop_steps[0] + hard.stop_steps[1] == 0 ? "yes" : "no");
        check_true("placeholder stop: never engaged in the default unsprung 1 m drop",
                   hard.stop_steps[0] + hard.stop_steps[1] == 0 && hard.stop_max[0] == 0 && hard.stop_max[1] == 0);
        check_true("placeholder stop: travel past rest below 0.25 m", hard.travel[0] < 0.25 && hard.travel[1] < 0.25);
    }

    // A: the car against the independent quarter-car, with the stop's test values (pure heave, lf = lr)
    {
        const ref_result r03 = reference(0.3), r1 = reference(1.0);
        std::printf("reference: lift 0.3 m travel %.3f mm stop %.1f N; lift 1 m travel %.3f mm stop %.1f N\n",
                    r03.travel * 1e3, r03.stop, r1.travel * 1e3, r1.stop);
        check_near("reference 0.3 m: travel 74.234 mm", r03.travel * 1e3, 74.234, 0.05);
        check_true("reference 0.3 m: the stop never engages", r03.stop == 0);
        check_near("reference 1 m: travel 130.992 mm", r1.travel * 1e3, 130.992, 0.05);
        check_near("reference 1 m: peak stop force 7996.1 N", r1.stop, 7996.1, 5.0);

        four_wheel_params q;
        q.vertical = vertical_model::unsprung;
        q.lf = q.lr = 1.25;
        rebuild_tyres(q);
        set_test_stop(q);
        const landing a = drop(q, 0.3, dt, 1.5);
        const landing b = drop(q, 1.0, dt, 1.5);
        const landing bf = drop(q, 1.0, 0.5 * dt, 1.5);
        const Real err_a = std::fabs(a.travel[0] - r03.travel);
        const Real err_b = std::fabs(b.travel[0] - r1.travel), ferr_b = std::fabs(b.stop_max[0] / r1.stop - 1);
        const Real err_f = std::fabs(bf.travel[0] - r1.travel), ferr_f = std::fabs(bf.stop_max[0] / r1.stop - 1);
        std::printf("car vs reference, FL: 0.3 m travel %.2f vs %.2f mm; 1 m 1 ms travel %.2f vs %.2f mm, stop %.1f vs "
                    "%.1f N (%.2f%%); 0.5 ms travel error %.2f mm, stop error %.2f%%\n",
                    a.travel[0] * 1e3, r03.travel * 1e3, b.travel[0] * 1e3, r1.travel * 1e3, b.stop_max[0], r1.stop,
                    100 * ferr_b, err_f * 1e3, 100 * ferr_f);
        check_true("car vs reference 0.3 m, 1 ms: travel within 1 mm", err_a < 1e-3);
        check_true("car vs reference 0.3 m, 1 ms: the stop force is exactly 0 at every step",
                   a.stop_max[0] == 0 && a.stop_steps[0] == 0);
        check_true("car vs reference 1 m, 1 ms: travel within 1 mm", err_b < 1e-3);
        check_true("car vs reference 1 m, 1 ms: peak stop force within 3%", ferr_b < 0.03);
        check_true("car vs reference 1 m, 0.5 ms: travel error no larger than at 1 ms", err_f <= err_b);
        check_true("car vs reference 1 m, 0.5 ms: stop force error no larger than at 1 ms", ferr_f <= ferr_b);
    }

    // B: the landing report on the default car with the stop's test values
    {
        four_wheel_params t;
        t.vertical = vertical_model::unsprung;
        set_test_stop(t);
        const char *const axle_name[2] = {"front", "rear"};
        for (const Real lift : {0.3, 1.0})
        {
            const landing l = drop(t, lift, dt, 4.0);
            char tag[48];
            std::snprintf(tag, sizeof tag, "stop test, %.1f m", lift);
            char what[200];
            for (int a = 0; a < 2; ++a)
            {
                std::printf("%s: %s travel past rest %.2f mm, bump stop peak %.1f N over %ld corner-steps, peak tyre "
                            "load %.0f N (%.2f x nominal %.0f N)\n",
                            tag, axle_name[a], l.travel[a] * 1e3, l.stop_max[a], l.stop_steps[a], l.peak[a],
                            l.peak[a] / t.front_tyre.nominal_load, t.front_tyre.nominal_load);
                if (lift < 0.5)
                {
                    std::snprintf(what, sizeof what, "%s: %s travel below 0.10 m", tag, axle_name[a]);
                    check_true(what, l.travel[a] < 0.10);
                    std::snprintf(what, sizeof what, "%s: %s bump stop force exactly 0 at every step", tag, axle_name[a]);
                    check_true(what, l.stop_max[a] == 0 && l.stop_steps[a] == 0);
                }
                else
                {
                    std::snprintf(what, sizeof what, "%s: %s bump stop engaged over 10 corner-steps", tag, axle_name[a]);
                    check_true(what, l.stop_steps[a] > 10);
                    std::snprintf(what, sizeof what, "%s: %s bump stop peak above 1000 N", tag, axle_name[a]);
                    check_true(what, l.stop_max[a] > 1000);
                }
            }
            std::snprintf(what, sizeof what, "%s: the tyre load is never below 0", tag);
            check_true(what, l.fz_min >= 0);
            std::snprintf(what, sizeof what, "%s: the stored energy is never above its starting value", tag);
            check_true(what, l.never_gained);
            std::snprintf(what, sizeof what, "%s: the damper rate is never below 0", tag);
            check_true(what, l.damper_min >= 0);
            std::printf("%s: smallest damper rate %.3g W, largest step-to-step rise of the energy %.2f J, peak stored "
                        "energy strut %.1f J tyre %.1f J droop stop %.1f J bump stop %.1f J\n",
                        tag, l.damper_min, l.rise_max, l.e_strut, l.e_tyre, l.e_droop, l.e_bump);
            check_settled(tag, t, l);
            const four_wheel_forces f = evaluate(l.end, t, four_wheel_input{});
            bool clear = true;
            for (std::size_t k = 0; k < kWheels; ++k)
                clear = clear && f.corner[k].stop == 0;
            std::snprintf(what, sizeof what, "%s: the stop force is exactly 0 at the end", tag);
            check_true(what, clear);
            std::printf("%s: body vertical speed at the end %.3e m/s\n", tag, l.end.body.vel.z);
        }
    }

    return finish("landing");
}
