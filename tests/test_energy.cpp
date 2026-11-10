#include <algorithm>
#include <cmath>
#include <cstdio>
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

    return finish("energy");
}
