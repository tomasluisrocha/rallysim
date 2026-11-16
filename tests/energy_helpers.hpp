#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include "runner/scenario_file.hpp"
#include "tests/check.hpp"

// Helpers shared by the energy tests: the run with its books, the test cars and the convergence checks.

using namespace rallysim;

// The run with its books, at time step dt (the scenario's duration kept).
struct booked
{
    four_wheel_state end;
    energy_books books;
    stored_energy e_end;
};

inline booked run_booked(four_wheel_run r, Real dt)
{
    const Real duration = r.steps * r.dt;
    r.dt = dt;
    r.steps = std::lround(duration / dt);
    energy_books books(r.initial, r.params);
    const four_wheel_state end = run(r, nullptr, &books);
    return {end, books, energy_of(end, r.params)};
}

// The energy that flows in a run: the most it stores at the start or end, or the work put in or taken out.
inline Real energy_scale(const booked &b)
{
    const energy_work &w = b.books.work();
    return std::max({b.books.start(), b.e_end.total(), w.drive, w.brake + w.damper + w.tyre});
}

// A car at rest on its springs, lifted `lift` above where it settles and let go: no drive, no brake.
inline four_wheel_run bounce(Real lift, long steps)
{
    four_wheel_run r;
    r.params.vertical = vertical_model::springs;
    r.initial = four_wheel_initial(r.params, 0.0);
    r.initial.body.pos.z += lift;
    r.script.keys = {{0.0, four_wheel_input{}}};
    r.steps = steps;
    return r;
}

// The unsprung car of the lifted-wheel test in test_four_wheel_unsprung: a high CG, a stiff front and 10 mm of
// front droop, turned hard enough that the inner front wheel hangs on its droop stop.
inline four_wheel_run lift_run()
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
inline four_wheel_run unsprung_scenario(const char *name)
{
    four_wheel_run r = four_wheel_scenario(name);
    r.params.vertical = vertical_model::unsprung;
    r.initial = four_wheel_initial(r.params, r.initial.body.vel.x);
    return r;
}

// Residuals at 2, 1 and 0.5 ms, and whether each halving of dt halves the residual (ratio 0.4 to 0.6).
inline void check_halves(const char *name, const four_wheel_run &scenario)
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
inline void check_shrinks(const char *name, const four_wheel_run &scenario)
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
inline four_wheel_run unsprung_drop(Real height, F tune)
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

// The scenario `name` in the unsprung model with the suspension geometry set on its car.
inline four_wheel_run geometry_scenario(const char *name)
{
    four_wheel_run r = unsprung_scenario(name);
    four_wheel_params &p = r.params;
    for (corner_params *c : {&p.front_corner, &p.rear_corner})
    {
        c->motion_ratio = 0.8;
        c->motion_ratio_gain = 1;
        c->camber = -0.02;
        c->camber_gain = -0.35;
        c->toe = 0.002;
        c->toe_gain = 0.05;
        c->roll_centre_gain = -0.5;
    }
    p.front_corner.roll_centre = 0.08;
    p.rear_corner.roll_centre = 0.12;
    p.front_tyre.camber_thrust = p.rear_tyre.camber_thrust = 0.1;
    p.ackermann = 1;
    r.initial = four_wheel_initial(p, r.initial.body.vel.x);
    return r;
}

inline void check_run(const char *name, const four_wheel_run &r, Real tolerance)
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
