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
        for (long i = 0; i < steps; ++i)
        {
            const four_wheel_forces f = evaluate(s, p, none);
            books.add(rates_of(s, p, f), dt);
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
            l.never_gained = l.never_gained && energy_of(s, p).total() <= e0 * (1 + 1e-12);
            l.lowest = std::min(l.lowest, s.body.pos.z);
        }
        l.residual = books.residual(energy_of(s, p));
        l.end = s;
        return l;
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
    }

    return finish("landing");
}
