#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include "runner/scenario_file.hpp"
#include "tests/check.hpp"
#include "tests/energy_helpers.hpp"

using namespace rallysim;

// Energy accounting for the unsprung scenarios left out of the other energy tests, and the suspension geometry (see test_energy.cpp).

int main()
{
    // the scenarios the checks above leave out, in the unsprung model: showcase, launch, handbrake, hairpin. Each starts
    // at rest at its ride height (unsprung_scenario), so the showcase run here is not the jump.
    for (const char *name : {"fw_showcase", "fw_launch", "fw_handbrake", "fw_hairpin"})
        check_run((std::string(name) + ", unsprung").c_str(), unsprung_scenario(name), 0.005);

    // the jump itself: fw_showcase from its own lifted, pitched start
    check_run("fw_showcase, unsprung, from its own start", four_wheel_scenario("fw_showcase", {{"vertical", "unsprung"}}),
              0.005);

    // the shipped scenarios, unsprung, never reach a stop, and their struts stay short of the 0.10 m default bump gap
    for (const char *name : {"fw_brake", "fw_circle", "fw_drop", "fw_hairpin", "fw_handbrake", "fw_lane", "fw_launch", "fw_ramp",
                             "fw_showcase"})
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
        check_true((std::string(name) + ", unsprung: strut travel past rest stays below 0.095 m").c_str(),
                   peak_travel < 0.095); // under the 0.10 m gap; the largest is 87.3 mm, on fw_showcase
    }

    // a motion ratio of 0.7 with a gain of 2 /m: the strut's energy is taken at its own compression, which moves
    // by the ratio times the wheel travel. The largest residual during a 0.3 m drop shrinks with dt.
    {
        const four_wheel_run drop = unsprung_drop(0.3, [](corner_params &c)
                                                  {
                                                      c.motion_ratio = 0.7;
                                                      c.motion_ratio_gain = 2;
                                                  });
        const four_wheel_input none{};
        auto largest_residual = [&](Real dt)
        {
            four_wheel_state s = drop.initial;
            energy_books books(s, drop.params);
            Real worst = 0;
            const long steps = std::lround(3.0 / dt);
            for (long i = 0; i < steps; ++i)
            {
                books.add(rates_of(s, drop.params, evaluate(s, drop.params, none)), dt);
                step(s, drop.params, none, dt);
                worst = std::max(worst, std::fabs(books.residual(energy_of(s, drop.params))));
            }
            return worst;
        };
        const Real r2 = largest_residual(0.002), r1 = largest_residual(0.001), r05 = largest_residual(0.0005);
        std::printf("motion ratio 0.7, gain 2, 0.3 m drop: largest residual %.1f J at 2 ms, %.1f at 1 ms, %.1f at 0.5 ms\n",
                    r2, r1, r05);
        check_true("motion ratio drop: each halving of dt leaves under 0.6 of the largest residual",
                   r1 < 0.6 * r2 && r05 < 0.6 * r1);
    }

    // roll centres 0.30 on both axles, a step of steer at 20 m/s: the jacking force and the patch's sideways speed
    // do work on the car that must match, or the residual stops halving with dt
    {
        four_wheel_run r;
        r.params.vertical = vertical_model::unsprung;
        r.params.front_corner.roll_centre = r.params.rear_corner.roll_centre = 0.30;
        r.initial = four_wheel_initial(r.params, 20.0);
        r.script.keys = {{0.0, four_wheel_input{}}, {0.5, four_wheel_input{}},
                         {0.6, four_wheel_input{0.04, 0.0, 0.0, 0.0}}};
        r.steps = 3000;
        check_halves("roll centres 0.30, steer step", r);
    }

    // the shipped unsprung scenarios with the whole geometry set: the books close and the residual halves with dt
    for (const char *name : {"fw_lane", "fw_circle", "fw_hairpin", "fw_handbrake", "fw_brake", "fw_launch"})
    {
        const four_wheel_run r = geometry_scenario(name);
        const std::string label = std::string(name) + ", unsprung, geometry";
        check_run(label.c_str(), r, 0.005);
        check_halves(label.c_str(), r);
    }

    return finish("energy_scenarios");
}
