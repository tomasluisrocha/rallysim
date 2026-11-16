#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include "runner/scenario_file.hpp"
#include "tests/check.hpp"
#include "tests/energy_helpers.hpp"

using namespace rallysim;

// Energy accounting for the unsprung car: anti-roll bar, stops, dampers and tyre damping (see test_energy.cpp).

int main()
{
    // The anti-roll bar stores elastic energy while the car rolls. Sample the largest residual during a rolled
    // recovery: at the end the bar may have unloaded, so an endpoint-only check would miss the term.
    {
        four_wheel_run r;
        r.params.vertical = vertical_model::unsprung;
        r.params.front_corner.antiroll_rate = 20000;
        r.params.rear_corner.antiroll_rate = 10000;
        four_wheel_pose pose{};
        pose.position = {0, 0, r.params.cg_height};
        pose.roll = 0.03;
        r.initial = four_wheel_start(r.params, pose);
        r.script.keys = {{0.0, four_wheel_input{}}};
        r.steps = 3000;

        auto peak_residual = [&](Real dt)
        {
            four_wheel_state s = r.initial;
            energy_books books(s, r.params);
            Real peak = 0;
            const long steps = std::lround(r.steps * r.dt / dt);
            for (long i = 0; i < steps; ++i)
            {
                const four_wheel_input in = driver_input(r, s, i * dt);
                books.add(rates_of(s, r.params, evaluate(s, r.params, in)), dt);
                step(s, r.params, in, dt);
                peak = std::max(peak, std::fabs(books.residual(energy_of(s, r.params))));
            }
            return peak;
        };
        const Real r2 = peak_residual(0.002), r1 = peak_residual(0.001), r05 = peak_residual(0.0005);
        std::printf("anti-roll, rolled recovery: largest residual %.2f J at 2 ms, %.2f at 1 ms, %.2f at 0.5 ms "
                    "(ratios %.3f, %.3f)\n",
                    r2, r1, r05, r1 / r2, r05 / r1);
        check_true("anti-roll, rolled recovery: residual is below 4 J at 0.5 ms", r05 < 4);
        check_true("anti-roll, rolled recovery: peak residual shrinks with dt",
                   r1 / r2 > 0.4 && r1 / r2 < 0.6 && r05 / r1 > 0.4 && r05 / r1 < 0.6);
    }

    // the same convergence in the unsprung model
    for (const char *name : {"fw_lane", "fw_brake"})
    {
        four_wheel_run scenario = four_wheel_scenario(name);
        scenario.params.vertical = vertical_model::unsprung;
        scenario.initial = four_wheel_initial(scenario.params, scenario.initial.body.vel.x);
        const booked coarse = run_booked(scenario, 0.002), fine = run_booked(scenario, 0.001),
                     finer = run_booked(scenario, 0.0005);
        const Real r2 = coarse.books.residual(coarse.e_end), r1 = fine.books.residual(fine.e_end),
                   r05 = finer.books.residual(finer.e_end);
        std::printf("convergence, %s, unsprung: residual %.3f J at 2 ms, %.3f at 1 ms, %.3f at 0.5 ms (ratios %.3f, %.3f)\n",
                    name, r2, r1, r05, r1 / r2, r05 / r1);
        char what[200];
        std::snprintf(what, sizeof what, "convergence, %s, unsprung: halving dt halves the residual (ratio 0.4 to 0.6, twice)", name);
        check_true(what, r1 / r2 > 0.4 && r1 / r2 < 0.6 && r05 / r1 > 0.4 && r05 / r1 < 0.6);
    }

    // The shipped unsprung scenarios with bars: the force must be the derivative of the stored bar energy.
    for (const char *name : {"fw_lane", "fw_circle", "fw_hairpin", "fw_handbrake", "fw_brake", "fw_launch"})
    {
        four_wheel_run r = four_wheel_scenario(name);
        r.params.vertical = vertical_model::unsprung;
        r.params.front_corner.antiroll_rate = 20000;
        r.params.rear_corner.antiroll_rate = 10000;
        r.initial = four_wheel_initial(r.params, r.initial.body.vel.x);
        check_run((std::string(name) + ", unsprung, anti-roll").c_str(), r, 0.005);
        check_halves((std::string(name) + ", unsprung, anti-roll").c_str(), r);
    }

    // the droop stop: the lifted inner front wheel hangs on it for seconds, so its energy is in the books
    {
        const four_wheel_run r = lift_run();
        four_wheel_state s = r.initial;
        Real deepest = 0;
        long engaged = 0;
        for (long i = 0; i < r.steps; ++i)
        {
            const four_wheel_forces f = evaluate(s, r.params, driver_input(r, s, i * r.dt));
            deepest = std::min(deepest, f.corner[FL].stop);
            engaged += f.corner[FL].stop < 0;
            step(s, r.params, driver_input(r, s, i * r.dt), r.dt);
        }
        std::printf("lift: the droop stop is engaged for %ld steps, at most %.1f N\n", engaged, deepest);
        check_true("lift: the droop stop is engaged for over 2 s", engaged > 2000 && deepest < -300);
        check_run("lift, droop stop engaged", r, 0.005);
        check_halves("lift, droop stop engaged", r);
    }

    // the bar on a lifted wheel: its force follows the strut compressions, negative ones included, so it still
    // acts (and its energy still counts) while the inner front wheel hangs off the ground with a slack strut
    {
        four_wheel_run r = lift_run();
        r.params.front_corner.antiroll_rate = 20000;
        four_wheel_state s = r.initial;
        long slack = 0;
        Real smallest = 1e9;
        for (long i = 0; i < r.steps; ++i)
        {
            const four_wheel_forces f = evaluate(s, r.params, driver_input(r, s, i * r.dt));
            if (f.fz[FL] == 0 && f.corner[FL].deflection < 0)
            {
                ++slack;
                smallest = std::min(smallest, f.corner[FL].deflection);
                check_near("lift, bar: force from the strut compressions", f.corner[FL].antiroll,
                           20000 * (f.corner[FL].deflection - f.corner[FR].deflection), 1e-9);
            }
            step(s, r.params, driver_input(r, s, i * r.dt), r.dt);
        }
        std::printf("lift, bar: FL off the ground with a slack strut for %ld steps, deflection down to %.4f m\n",
                    slack, smallest);
        check_true("lift, bar: over 1000 steps off the ground with a slack strut", slack > 1000);
        check_run("lift, anti-roll bar", r, 0.005);
        check_halves("lift, anti-roll bar", r);
    }

    // tyre damping: a landing from 0.2 m (tyre compression speeds of metres per second) with a nonzero coefficient.
    // The dampers' book takes more out than with none, and the residual still obeys the bound and halves with dt.
    {
        auto landing = [](Real tyre_damping)
        { return unsprung_drop(0.2, [tyre_damping](corner_params &c) { c.tyre_damping = tyre_damping; }); };
        const four_wheel_run damped = landing(2000), plain = landing(0);
        check_run("landing, tyre damping 2000", damped, 0.005);
        const Real with = run_booked(damped, damped.dt).books.work().damper,
                   without = run_booked(plain, plain.dt).books.work().damper;
        std::printf("landing: damper work %.1f J with tyre damping 2000, %.1f J with none\n", with, without);
        check_true("landing: tyre damping takes more energy out through the dampers", with > without + 1);
        check_halves("landing, tyre damping 2000", damped);
    }

    // rebound damping three times the bump damping: the books still close, and the damper only takes energy out
    {
        four_wheel_run lane = unsprung_scenario("fw_lane");
        lane.params.front_corner.rebound_ratio = lane.params.rear_corner.rebound_ratio = 3;
        check_run("fw_lane, unsprung, rebound 3", lane, 0.005);
        check_halves("fw_lane, unsprung, rebound 3", lane);

        const four_wheel_run r = unsprung_drop(0.2,
                                               [](corner_params &c)
                                               {
                                                   c.tyre_damping = 0;
                                                   c.rebound_ratio = 3;
                                               });
        check_run("landing, rebound 3", r, 0.005);
        check_shrinks("landing, rebound 3", r);

        four_wheel_state s = r.initial;
        long rebound_steps = 0;
        Real min_power = 0;
        bool never_gives = true, rebound_seen = false;
        for (long i = 0; i < r.steps; ++i)
        {
            const four_wheel_forces f = evaluate(s, r.params, driver_input(r, s, i * r.dt));
            for (std::size_t k = 0; k < kWheels; ++k)
            {
                const corner_load &l = f.corner[k];
                never_gives = never_gives && l.damper * l.deflection_rate >= 0;
                min_power = std::min(min_power, l.damper * l.deflection_rate);
                const bool reb = l.deflection_rate < 0 && l.damper < 0;
                rebound_seen = rebound_seen || reb;
                rebound_steps += reb;
            }
            step(s, r.params, driver_input(r, s, i * r.dt), r.dt);
        }
        std::printf("landing, rebound 3: %ld corner-steps on the rebound side; smallest damper power %.3f W\n", rebound_steps,
                    min_power);
        check_true("landing, rebound 3: every damper power is >= 0 at every step", never_gives);
        check_true("landing, rebound 3: the rebound side was exercised", rebound_seen);
    }

    // the bump stop: a 1 m unsprung drop with the stop moved to 0.10 m past rest (test values, not car data), so the
    // landing runs into it. The books are checked during the landing, while the stop holds energy, not only at the
    // end: the largest residual over the run is a small part of the energy that flows and halves with dt.
    {
        const four_wheel_run r = unsprung_drop(1.0,
                                               [](corner_params &c)
                                               {
                                                   c.tyre_damping = 0;
                                                   c.bump_travel = 0.10;
                                               });
        struct drop_run
        {
            Real peak_force = 0, peak_residual = 0, peak_stop_energy = 0, scale = 0;
            long engaged = 0;
        };
        // the run stepped by hand at time step dt (3 s), its books kept step by step
        auto drop = [](const four_wheel_run &r, Real dt)
        {
            const four_wheel_params &p = r.params;
            const long steps = std::lround(r.steps * r.dt / dt);
            four_wheel_state s = r.initial;
            energy_books books(s, p);
            drop_run d;
            for (long i = 0; i < steps; ++i)
            {
                const four_wheel_input in = driver_input(r, s, i * dt);
                const four_wheel_forces f = evaluate(s, p, in);
                for (std::size_t k = 0; k < kWheels; ++k)
                {
                    d.peak_force = std::max(d.peak_force, f.corner[k].stop);
                    d.engaged += f.corner[k].stop > 0;
                }
                books.add(rates_of(s, p, f), dt);
                step(s, p, in, dt);
                d.peak_residual = std::max(d.peak_residual, std::fabs(books.residual(energy_of(s, p))));
                Real stop_energy = 0; // each corner's compression as energy_of computes it
                for (std::size_t k = 0; k < kWheels; ++k)
                {
                    const corner_params &c = p.corner(k);
                    const Real mount = p.static_deflection(k) - contact_of(p, s.body, k, 0.0).deflection;
                    const Real x = kinematics_of(p, k, mount, 0.0, s.corners[k]).strut;
                    stop_energy += bump_stop_energy(x - (p.strut_static_deflection(k) + c.bump_travel), c.bump_stop_rate,
                                                    c.bump_stop_length);
                }
                d.peak_stop_energy = std::max(d.peak_stop_energy, stop_energy);
            }
            const energy_work &w = books.work();
            d.scale = std::max({books.start(), energy_of(s, p).total(), w.drive, w.brake + w.damper + w.tyre});
            return d;
        };
        const drop_run d2 = drop(r, 0.002), d1 = drop(r, 0.001), d05 = drop(r, 0.0005);
        std::printf("drop, bump stop: engaged for %ld corner-steps, peak %.1f N, peak stop energy %.1f J\n", d1.engaged,
                    d1.peak_force, d1.peak_stop_energy);
        std::printf("drop, bump stop: largest residual %.3f J at 2 ms, %.3f at 1 ms, %.3f at 0.5 ms (ratios %.3f, %.3f)\n",
                    d2.peak_residual, d1.peak_residual, d05.peak_residual, d1.peak_residual / d2.peak_residual,
                    d05.peak_residual / d1.peak_residual);
        check_true("drop, bump stop: the stop is engaged", d1.peak_force > 1000 && d1.engaged > 10);
        check_true("drop, bump stop: the stop stores over 100 J", d1.peak_stop_energy > 100);
        check_true("drop, bump stop: the largest residual during the run is within 0.50% of the energy that flows",
                   d1.peak_residual <= 0.005 * d1.scale);
        const Real q1 = d1.peak_residual / d2.peak_residual, q05 = d05.peak_residual / d1.peak_residual;
        check_true("drop, bump stop: halving dt halves the largest residual (ratio 0.4 to 0.6, twice)",
                   q1 > 0.4 && q1 < 0.6 && q05 > 0.4 && q05 < 0.6);
        check_run("drop, bump stop", r, 0.005);
        check_shrinks("drop, bump stop", r);

        // the same drop with the bump stop's own rate and length set apart from the droop stop's rate (500000)
        const four_wheel_run own = unsprung_drop(1.0,
                                                 [](corner_params &c)
                                                 {
                                                     c.tyre_damping = 0;
                                                     c.bump_travel = 0.10;
                                                     c.bump_stop_rate = 300000;
                                                     c.bump_stop_length = 0.02;
                                                 });
        const drop_run o1 = drop(own, 0.001), o05 = drop(own, 0.0005);
        std::printf("drop, own bump stop: engaged for %ld corner-steps, peak %.1f N, peak stop energy %.1f J\n", o1.engaged,
                    o1.peak_force, o1.peak_stop_energy);
        std::printf("drop, own bump stop: largest residual %.3f J at 1 ms (of %.0f J), %.3f at 0.5 ms (ratio %.3f)\n",
                    o1.peak_residual, o1.scale, o05.peak_residual, o05.peak_residual / o1.peak_residual);
        check_true("drop, own bump stop: the stop is engaged", o1.peak_force > 1000 && o1.engaged > 10);
        check_true("drop, own bump stop: the largest residual during the run is within 0.50% of the energy that flows",
                   o1.peak_residual <= 0.005 * o1.scale);
        const Real qo = o05.peak_residual / o1.peak_residual;
        check_true("drop, own bump stop: halving dt halves the largest residual (ratio 0.4 to 0.6)", qo > 0.4 && qo < 0.6);

        // the same drop with a motion ratio of 0.8: the stop's energy is taken at the strut's compression. Catches it
        // taken at the wheel's travel.
        const four_wheel_run ratio = unsprung_drop(1.0,
                                                   [](corner_params &c)
                                                   {
                                                       c.tyre_damping = 0;
                                                       c.bump_travel = 0.10;
                                                       c.bump_stop_length = 0.03;
                                                       c.bump_stop_rate = 500000;
                                                       c.motion_ratio = 0.8;
                                                   });
        const drop_run m2 = drop(ratio, 0.002), m1 = drop(ratio, 0.001), m05 = drop(ratio, 0.0005);
        std::printf("drop, bump stop, ratio 0.8: engaged for %ld corner-steps, peak %.1f N, peak stop energy %.1f J; largest "
                    "residual %.3f J at 2 ms, %.3f at 1 ms, %.3f at 0.5 ms\n",
                    m1.engaged, m1.peak_force, m1.peak_stop_energy, m2.peak_residual, m1.peak_residual, m05.peak_residual);
        check_true("drop, bump stop, ratio 0.8: the stop is engaged", m1.peak_force > 1000 && m1.engaged > 10);
        check_true("drop, bump stop, ratio 0.8: the diagnostic reads the stop holding energy", m1.peak_stop_energy > 1);
        check_true("drop, bump stop, ratio 0.8: each halving of dt leaves under 0.6 of the largest residual",
                   m1.peak_residual < 0.6 * m2.peak_residual && m05.peak_residual < 0.6 * m1.peak_residual);
    }

    // the bar's energy with a ratio of 0.7 on both axles: with a front bar of 20000 N/m and FL raised 0.013 m at rest,
    // the slope of the stored springs energy against FL's height, with the bar less without it, is 20000 x 0.013 N.
    // Catches the bar's energy taken on strut compression (127.4 N).
    {
        auto bar_slope = [](Real bar)
        {
            four_wheel_params p;
            p.vertical = vertical_model::unsprung;
            p.front_corner.motion_ratio = p.rear_corner.motion_ratio = 0.7;
            p.front_corner.antiroll_rate = bar;
            const Real h = 1e-6;
            auto springs = [&](Real dz)
            {
                four_wheel_state s = four_wheel_initial(p, 0.0);
                s.corners[FL].z += 0.013 + dz;
                return energy_of(s, p).springs;
            };
            return (springs(h) - springs(-h)) / (2 * h);
        };
        const Real slope = bar_slope(20000) - bar_slope(0);
        std::printf("bar energy, ratio 0.7: slope %.6f N (kb x 0.013 = 260 N)\n", slope);
        check_near("bar energy, ratio 0.7: slope of the bar's energy is kb x travel", slope, 20000 * 0.013, 0.01);
    }

    return finish("energy_unsprung");
}
