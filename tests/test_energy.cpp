#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include "runner/scenario_file.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// Energy accounting for the four-wheel car: the energy stored in the car changes only by the work
// done on it, so (E(t) - E(0)) - (drive - brake - damper - tyre work) stays near zero (vehicle/four_wheel_energy.hpp).
//
// The residual is not exactly zero: the books add each step's rates at the start of the step, while step()
// applies the tyre forces averaged over the wheel's sub-steps, an error of first order in dt. So the checks are:
// the residual is a small fraction of the energy that flows in the run; halving dt halves it (an integration error
// shrinks, a physics bug such as a wrong sign or a missing force would not); and the brakes, dampers and tyres only
// ever take energy out over a run. Measured when written: worst residual 0.33% of the energy flow (fw_circle, the
// longest run, 20 s), every scenario's residual halving with dt (ratio 0.49 to 0.50).

namespace
{
    // The run with its books, at time step dt (the scenario's duration kept).
    struct booked
    {
        four_wheel_state end;
        energy_books books;
        stored_energy e_end;
    };

    booked run_booked(four_wheel_run r, Real dt)
    {
        const Real duration = r.steps * r.dt;
        r.dt = dt;
        r.steps = std::lround(duration / dt);
        energy_books books(r.initial, r.params);
        const four_wheel_state end = run(r, nullptr, &books);
        return {end, books, energy_of(end, r.params)};
    }

    // The energy that flows in a run: the most it stores at the start or end, or the work put in or taken out.
    Real energy_scale(const booked &b)
    {
        const energy_work &w = b.books.work();
        return std::max({b.books.start(), b.e_end.total(), w.drive, w.brake + w.damper + w.tyre});
    }

    // A car at rest on its springs, lifted `lift` above where it settles and let go: no drive, no brake.
    four_wheel_run bounce(Real lift, long steps)
    {
        four_wheel_run r;
        r.initial = four_wheel_initial(r.params, 0.0);
        r.initial.body.pos.z += lift;
        r.script.keys = {{0.0, four_wheel_input{}}};
        r.steps = steps;
        return r;
    }

    // The unsprung car of the lifted-wheel test in test_four_wheel_unsprung: a high CG, a stiff front and 10 mm of
    // front droop, turned hard enough that the inner front wheel hangs on its droop stop.
    four_wheel_run lift_run()
    {
        four_wheel_run r;
        r.params.vertical = vertical_model::unsprung;
        r.params.set_tyre_curve(tyre_curve::linear);
        r.params.front_corner.spring_rate = 300000;
        r.params.front_corner.droop = 0.01;
        r.params.rear_corner.spring_rate = 30000;
        r.params.cg_height = 0.85;
        r.initial = four_wheel_initial(r.params, 18.0);
        r.hold_speed = constant_script<Real>(18.0);
        r.steps = 6000;
        r.script.keys = {{0.0, four_wheel_input{}}, {0.5, four_wheel_input{}},
                         {1.5, four_wheel_input{0.09, 0.0, 0.0, 0.0}}, {6.0, four_wheel_input{0.09, 0.0, 0.0, 0.0}}};
        return r;
    }

    // The scenario `name` in the unsprung model.
    four_wheel_run unsprung_scenario(const char *name)
    {
        four_wheel_run r = four_wheel_scenario(name);
        r.params.vertical = vertical_model::unsprung;
        r.initial = four_wheel_initial(r.params, r.initial.body.vel.x);
        return r;
    }

    // Residuals at 2, 1 and 0.5 ms, and whether each halving of dt halves the residual (ratio 0.4 to 0.6).
    void check_halves(const char *name, const four_wheel_run &scenario)
    {
        const booked coarse = run_booked(scenario, 0.002), fine = run_booked(scenario, 0.001),
                     finer = run_booked(scenario, 0.0005);
        const Real r2 = coarse.books.residual(coarse.e_end), r1 = fine.books.residual(fine.e_end),
                   r05 = finer.books.residual(finer.e_end);
        std::printf("convergence, %s: residual %.3f J at 2 ms, %.3f at 1 ms, %.3f at 0.5 ms (ratios %.3f, %.3f)\n", name, r2, r1,
                    r05, r1 / r2, r05 / r1);
        char what[200];
        std::snprintf(what, sizeof what, "convergence, %s: halving dt halves the residual (ratio 0.4 to 0.6, twice)", name);
        check_true(what, r1 / r2 > 0.4 && r1 / r2 < 0.6 && r05 / r1 > 0.4 && r05 / r1 < 0.6);
    }

    // Residuals at 2, 1 and 0.5 ms, and whether each halving of dt leaves under 0.6 of the residual: the rule for
    // unsprung drops without tyre damping, which converge faster than first order.
    void check_shrinks(const char *name, const four_wheel_run &scenario)
    {
        const booked coarse = run_booked(scenario, 0.002), fine = run_booked(scenario, 0.001),
                     finer = run_booked(scenario, 0.0005);
        const Real r2 = coarse.books.residual(coarse.e_end), r1 = fine.books.residual(fine.e_end),
                   r05 = finer.books.residual(finer.e_end);
        std::printf("convergence, %s: residual %.3f J at 2 ms, %.3f at 1 ms, %.3f at 0.5 ms (ratios %.3f, %.3f)\n", name, r2, r1,
                    r05, r1 / r2, r05 / r1);
        char what[200];
        std::snprintf(what, sizeof what, "convergence, %s: each halving of dt leaves under 0.6 of the residual", name);
        check_true(what, std::fabs(r1) < 0.6 * std::fabs(r2) && std::fabs(r05) < 0.6 * std::fabs(r1));
    }

    // The unsprung car at rest, body and wheels lifted `height` and let go, no input, 3000 steps. `tune` sets each
    // corner's parameters before the car is placed.
    template <class F>
    four_wheel_run unsprung_drop(Real height, F tune)
    {
        four_wheel_run r;
        r.params.vertical = vertical_model::unsprung;
        tune(r.params.front_corner);
        tune(r.params.rear_corner);
        r.initial = four_wheel_initial(r.params, 0.0);
        r.initial.body.pos.z += height;
        for (corner_state &w : r.initial.corners)
            w.z += height;
        r.script.keys = {{0.0, four_wheel_input{}}};
        r.steps = 3000;
        return r;
    }

    void check_run(const char *name, const four_wheel_run &r, Real tolerance)
    {
        const booked b = run_booked(r, r.dt);
        const energy_work &w = b.books.work();
        const Real residual = b.books.residual(b.e_end), scale = energy_scale(b);
        std::printf("%-26s E %9.0f -> %9.0f J; work: drive %8.0f brake %7.0f damper %6.0f tyre %8.0f; residual %8.2f J (%.3f%% of %.0f J)\n",
                    name, b.books.start(), b.e_end.total(), w.drive, w.brake, w.damper, w.tyre, residual,
                    100 * std::fabs(residual) / scale, scale);

        char what[200];
        std::snprintf(what, sizeof what, "%s: residual within %.2f%% of the energy that flows", name, 100 * tolerance);
        check_true(what, std::fabs(residual) <= tolerance * scale);
        std::snprintf(what, sizeof what, "%s: brakes, dampers and tyres only take energy out (net over the run)", name);
        check_true(what, w.brake >= 0 && w.damper >= 0 && w.tyre >= 0);
        std::snprintf(what, sizeof what, "%s: the drive only puts energy in", name);
        check_true(what, w.drive >= 0);
    }
}

int main()
{
    // every four-wheel scenario, as run (1 ms, springs, the car's tmeasy tyre)
    check_run("fw_circle", four_wheel_scenario("fw_circle"), 0.005);
    check_run("fw_lane", four_wheel_scenario("fw_lane"), 0.005);
    check_run("fw_brake", four_wheel_scenario("fw_brake"), 0.005);
    check_run("fw_launch", four_wheel_scenario("fw_launch"), 0.005);
    check_run("fw_handbrake", four_wheel_scenario("fw_handbrake"), 0.005);
    check_run("fw_hairpin", four_wheel_scenario("fw_hairpin"), 0.005);
    check_run("fw_showcase", four_wheel_scenario("fw_showcase"), 0.005); // a jump and a landing, at 0.5 ms

    // the planar model (gravity cancelled, no springs): only motion and wheel spin are stored
    {
        four_wheel_run r = four_wheel_scenario("fw_circle");
        r.params.vertical = vertical_model::planar;
        r.initial = four_wheel_initial(r.params, 6.0);
        check_run("fw_circle, planar", r, 0.005);
    }

    // the unsprung model (wheel masses on tyre springs): the same scenarios, the same bounds
    {
        const char *const names[3] = {"fw_circle", "fw_lane", "fw_brake"};
        for (const char *name : names)
        {
            four_wheel_run r = four_wheel_scenario(name);
            r.params.vertical = vertical_model::unsprung;
            r.initial = four_wheel_initial(r.params, r.initial.body.vel.x);
            check_run((std::string(name) + ", unsprung").c_str(), r, 0.005);
        }
    }

    // a free bounce: lifted 2 cm on the springs and let go, nothing driving or braking. The springs and gravity trade
    // energy back and forth; only the dampers (and the tyres' patch damping at rest) take any out, so the energy
    // the car loses is what they took.
    {
        const four_wheel_run r = bounce(0.02, 3000);
        check_run("free bounce", r, 0.005);
        const booked b = run_booked(r, r.dt);
        const energy_work &w = b.books.work();
        check_true("free bounce: no drive and no brake work", w.drive == 0 && w.brake == 0);
        check_true("free bounce: the dampers take energy out", w.damper > 0);
        check_true("free bounce: the car ends with less energy than it started with", b.e_end.total() < b.books.start());
    }

    // convergence: halving dt halves the residual (first order, like the step itself). A wrong sign or a missing
    // force would leave a part that does not shrink.
    for (const auto &[name, scenario] : {std::pair<const char *, four_wheel_run>{"fw_lane", four_wheel_scenario("fw_lane")},
                                         std::pair<const char *, four_wheel_run>{"fw_brake", four_wheel_scenario("fw_brake")}})
    {
        const booked coarse = run_booked(scenario, 0.002), fine = run_booked(scenario, 0.001),
                     finer = run_booked(scenario, 0.0005);
        const Real r2 = coarse.books.residual(coarse.e_end), r1 = fine.books.residual(fine.e_end),
                   r05 = finer.books.residual(finer.e_end);
        std::printf("convergence, %s: residual %.3f J at 2 ms, %.3f at 1 ms, %.3f at 0.5 ms (ratios %.3f, %.3f)\n", name, r2,
                    r1, r05, r1 / r2, r05 / r1);
        char what[200];
        std::snprintf(what, sizeof what, "convergence, %s: halving dt halves the residual (ratio 0.4 to 0.6, twice)", name);
        check_true(what, r1 / r2 > 0.4 && r1 / r2 < 0.6 && r05 / r1 > 0.4 && r05 / r1 < 0.6);
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
                    const Real x = p.strut_static_deflection(k) - mount + (s.corners[k].z - p.wheel_rest_height(k));
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
    }

    // the scenarios the checks above leave out, in the unsprung model: showcase, launch, handbrake, hairpin. Each starts
    // at rest at its ride height (unsprung_scenario), so the showcase run here is not the jump.
    for (const char *name : {"fw_showcase", "fw_launch", "fw_handbrake", "fw_hairpin"})
        check_run((std::string(name) + ", unsprung").c_str(), unsprung_scenario(name), 0.005);

    // the jump itself: fw_showcase from its own lifted, pitched start
    check_run("fw_showcase, unsprung, from its own start", four_wheel_scenario("fw_showcase", {{"vertical", "unsprung"}}),
              0.005);

    // the shipped scenarios, unsprung, never reach a stop, and their struts stay well short of the bump stop
    for (const char *name : {"fw_brake", "fw_circle", "fw_hairpin", "fw_handbrake", "fw_lane", "fw_launch", "fw_showcase"})
    {
        const four_wheel_run r = four_wheel_scenario(name, {{"vertical", "unsprung"}});
        const four_wheel_params &p = r.params;
        four_wheel_state s = r.initial;
        Real peak_stop = 0, peak_travel = 0;
        for (long i = 0; i < r.steps; ++i)
        {
            const four_wheel_input in = driver_input(r, s, i * r.dt);
            const four_wheel_forces f = evaluate(s, p, in);
            for (std::size_t k = 0; k < kWheels; ++k)
            {
                peak_stop = std::max(peak_stop, std::fabs(f.corner[k].stop));
                peak_travel = std::max(peak_travel, f.corner[k].deflection - p.strut_static_deflection(k));
            }
            step(s, p, in, r.dt);
        }
        std::printf("%s, unsprung: largest stop force %.1f N, largest strut travel past rest %.1f mm\n", name, peak_stop,
                    1000 * peak_travel);
        check_true((std::string(name) + ", unsprung: no stop is ever reached").c_str(), peak_stop == 0);
        check_true((std::string(name) + ", unsprung: strut travel past rest stays below 0.125 m").c_str(),
                   peak_travel < 0.125);
    }

    return finish("energy");
}
