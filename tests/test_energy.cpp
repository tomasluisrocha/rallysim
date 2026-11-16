#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include "runner/scenario_file.hpp"
#include "tests/check.hpp"
#include "tests/energy_helpers.hpp"

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


int main()
{
    // every four-wheel scenario, as run (1 ms, springs, the car's tmeasy tyre)
    check_run("fw_circle", four_wheel_scenario("fw_circle", {{"vertical", "springs"}}), 0.005);
    check_run("fw_lane", four_wheel_scenario("fw_lane", {{"vertical", "springs"}}), 0.005);
    check_run("fw_brake", four_wheel_scenario("fw_brake", {{"vertical", "springs"}}), 0.005);
    check_run("fw_launch", four_wheel_scenario("fw_launch", {{"vertical", "springs"}}), 0.005);
    check_run("fw_handbrake", four_wheel_scenario("fw_handbrake", {{"vertical", "springs"}}), 0.005);
    check_run("fw_hairpin", four_wheel_scenario("fw_hairpin", {{"vertical", "springs"}}), 0.005);
    check_run("fw_showcase", four_wheel_scenario("fw_showcase", {{"vertical", "springs"}}), 0.005); // a jump and a landing, at 0.5 ms

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
    for (const auto &[name, scenario] : {std::pair<const char *, four_wheel_run>{"fw_lane", four_wheel_scenario("fw_lane", {{"vertical", "springs"}})},
                                         std::pair<const char *, four_wheel_run>{"fw_brake", four_wheel_scenario("fw_brake", {{"vertical", "springs"}})}})
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
