#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>
#include "check.hpp"
#include "scenarios/scenario_file.hpp"
#include "vehicle/four_wheel_registry.hpp"
#include "vehicle/four_wheel_telemetry.hpp"

using namespace rallysim;

namespace
{
    Real value_of(four_wheel_params &p, const param_entry &e) { return e.field(p); }

    bool throws(const std::string &name, const std::string &value)
    {
        four_wheel_params p;
        try
        {
            apply(p, name, value);
        }
        catch (const std::invalid_argument &)
        {
            return true;
        }
        return false;
    }

    // The stiffness a tmeasy tyre has at `load`, along and across.
    Real along_at(const tyre_params &t, Real load) { return tmeasy_at(t.tmeasy_x, load, t.nominal_load).stiffness; }
    Real across_at(const tyre_params &t, Real load) { return tmeasy_at(t.tmeasy_y, load, t.nominal_load).stiffness; }

    // Both axles' tables give the tyre's own stiffnesses at that axle's load at rest.
    void check_fitted(const char *what, const four_wheel_params &p)
    {
        const Real tol = 1e-9 * 90000;
        check_near(what, along_at(p.front_tyre, p.static_load(FL)), p.front_tyre.longitudinal_stiffness, tol);
        check_near(what, across_at(p.front_tyre, p.static_load(FL)), p.front_tyre.cornering_stiffness, tol);
        check_near(what, along_at(p.rear_tyre, p.static_load(RL)), p.rear_tyre.longitudinal_stiffness, tol);
        check_near(what, across_at(p.rear_tyre, p.static_load(RL)), p.rear_tyre.cornering_stiffness, tol);
    }

    bool same_tables(const tyre_params &a, const tyre_params &b)
    {
        return a.tmeasy_x.nominal.stiffness == b.tmeasy_x.nominal.stiffness &&
               a.tmeasy_x.double_load.stiffness == b.tmeasy_x.double_load.stiffness &&
               a.tmeasy_y.nominal.stiffness == b.tmeasy_y.nominal.stiffness &&
               a.tmeasy_y.double_load.stiffness == b.tmeasy_y.double_load.stiffness;
    }
}

int main()
{
    const std::vector<param_entry> &table = four_wheel_registry();

    // ---- the table ------------------------------------------------------------------------------------------
    for (std::size_t i = 0; i < table.size(); ++i)
    {
        for (std::size_t j = i + 1; j < table.size(); ++j)
            check_true("registry: names are unique", std::string(table[i].name) != table[j].name);
        check_true("registry: find_param finds every entry", find_param(table[i].name) == &table[i]);
    }
    check_true("registry: an unknown name is not found", find_param("sprng_front") == nullptr);

    // every entry reaches its own field: a value written through one entry is read back by it and by no other
    // (a copy-pasted lambda pointing at the wrong field fails here)
    for (const param_entry &e : table)
    {
        four_wheel_params p;
        const four_wheel_params defaults;
        e.field(p) = 123456.75;
        check_near(e.name, value_of(p, e), 123456.75, 0);
        for (const param_entry &other : table)
            if (&other != &e)
            {
                four_wheel_params d = defaults;
                check_true(other.name, value_of(p, other) == value_of(d, other));
            }
    }

    // a name on telemetry line 1 means the same number in both places
    {
        four_wheel_params p;
        for (const auto &[key, value] : four_wheel_meta(p))
            if (const param_entry *e = find_param(key))
                check_near(e->name, e->field(p), value, 0);
        for (const char *name : {"mass", "lf", "spring_front", "mu_rear", "cx_front", "calpha_rear"})
        {
            bool on_line_1 = false;
            for (const auto &kv : four_wheel_meta(p))
                on_line_1 = on_line_1 || kv.first == name;
            check_true("registry: shares its names with telemetry line 1", on_line_1 && find_param(name));
        }
    }

    // ---- apply ----------------------------------------------------------------------------------------------
    {
        four_wheel_params p;
        apply(p, "spring_front", "60000");
        check_near("apply: sets the field", p.front_corner.spring_rate, 60000, 0);
        check_near("apply: and only that field", p.rear_corner.spring_rate, four_wheel_params{}.rear_corner.spring_rate, 0);
        apply(p, "mu_rear", "6.5e-1");
        check_near("apply: reads exponents", p.rear_tyre.friction_coefficient, 0.65, 0);
        apply(p, "lf", " 1.25");
        check_near("apply: leading spaces are fine", p.lf, 1.25, 0);
    }
    check_true("apply: unknown name throws", throws("sprng_front", "60000"));
    check_true("apply: text throws", throws("spring_front", "abc"));
    check_true("apply: trailing junk throws", throws("spring_front", "6e4abc"));
    check_true("apply: empty throws", throws("spring_front", ""));
    check_true("apply: infinity throws", throws("spring_front", "inf"));
    check_true("apply: nan throws", throws("spring_front", "nan"));
    check_true("apply: overflow throws", throws("spring_front", "1e999"));

    // ---- rebuild_tyres --------------------------------------------------------------------------------------
    {
        // the car as built is already fitted, so a rebuild only moves the tables by rounding (the scale factor is 1
        // to within an ulp), and rebuilding again and again does not drift
        four_wheel_params p;
        check_fitted("rebuild: the default car is fitted", p);
        const four_wheel_params before = p;
        rebuild_tyres(p);
        const auto moved = [&](const four_wheel_params &q)
        {
            Real worst = 0;
            for (const auto &[now, was] : {std::pair{&q.front_tyre, &before.front_tyre}, {&q.rear_tyre, &before.rear_tyre}})
                for (const auto &[a, b] : {std::pair{now->tmeasy_x.nominal.stiffness, was->tmeasy_x.nominal.stiffness},
                                          {now->tmeasy_x.double_load.stiffness, was->tmeasy_x.double_load.stiffness},
                                          {now->tmeasy_y.nominal.stiffness, was->tmeasy_y.nominal.stiffness},
                                          {now->tmeasy_y.double_load.stiffness, was->tmeasy_y.double_load.stiffness}})
                    worst = std::max(worst, std::fabs(a - b) / b);
            return worst;
        };
        std::printf("rebuild of the default car: tables moved by %.1e (relative)\n", moved(p));
        check_true("rebuild: leaves a fitted car unchanged but for rounding", moved(p) < 1e-15);
        for (int k = 0; k < 1000; ++k)
            rebuild_tyres(p);
        std::printf("after 1000 more rebuilds: %.1e\n", moved(p));
        check_true("rebuild: repeated rebuilds do not drift", moved(p) < 1e-14);
    }
    {
        // a heavier car: the tables no longer fit until the rebuild, then fit at the new loads
        four_wheel_params p;
        apply(p, "mass", "1500");
        check_true("rebuild: a heavier car's tyres are off before the rebuild",
                   across_at(p.front_tyre, p.static_load(FL)) > 1.01 * p.front_tyre.cornering_stiffness);
        rebuild_tyres(p);
        check_fitted("rebuild: a heavier car", p);
        check_near("rebuild: the front fits 30 kN/rad", across_at(p.front_tyre, p.static_load(FL)), 30000, 1e-6);
    }
    {
        // the CG moved back: more load on the rears, less on the fronts, both fitted
        four_wheel_params p;
        apply(p, "lf", "1.5");
        apply(p, "lr", "1.0");
        rebuild_tyres(p);
        check_fitted("rebuild: CG moved back", p);
        check_near("rebuild: rear load at rest", p.static_load(RL), 0.5 * 1200 * kGravity * 1.5 / 2.5, 1e-9);
    }
    {
        // a new stiffness takes effect on the tmeasy tables, and the linear curve reads it directly
        four_wheel_params p;
        apply(p, "calpha_front", "40000");
        apply(p, "cx_rear", "100000");
        rebuild_tyres(p);
        check_fitted("rebuild: new stiffnesses", p);
        check_near("rebuild: calpha_front reaches tmeasy", across_at(p.front_tyre, p.static_load(FL)), 40000, 1e-6);
        check_near("rebuild: rear cornering untouched", across_at(p.rear_tyre, p.static_load(RL)), 35000, 1e-6);
    }
    {
        // grip survives the rebuild, and the order of the settings does not matter
        four_wheel_params a, b;
        apply(a, "mu_front", "0.6");
        apply(a, "mass", "1500");
        apply(a, "lf", "1.4");
        rebuild_tyres(a);
        apply(b, "lf", "1.4");
        apply(b, "mass", "1500");
        apply(b, "mu_front", "0.6");
        rebuild_tyres(b);
        check_near("rebuild: keeps the grip", a.front_tyre.friction_coefficient, 0.6, 0);
        check_true("rebuild: any order gives the same tyres", same_tables(a.front_tyre, b.front_tyre) &&
                                                                 same_tables(a.rear_tyre, b.rear_tyre));
        check_true("rebuild: keeps the curve", a.front_tyre.curve == tyre_curve::tmeasy);
    }

    // ---- with_settings: the scenario's start state follows the new car -------------------------------------
    {
        const four_wheel_run plain = four_wheel_scenario("fw_brake");
        const four_wheel_run same = with_settings(plain, {});
        check_true("with_settings: no settings, no change", same.initial.body.pos.z == plain.initial.body.pos.z &&
                                                               same.initial.wheels[FL].omega == plain.initial.wheels[FL].omega);

        const four_wheel_run r = with_settings(plain, {{"cg_height", "0.6"}, {"wheel_radius", "0.33"}});
        check_near("with_settings: starts at the new ride height", r.initial.body.pos.z, 0.6, 0);
        check_near("with_settings: keeps the start speed", r.initial.body.vel.x, plain.initial.body.vel.x, 0);
        check_near("with_settings: wheels roll for the new radius", r.initial.wheels[RR].omega,
                   rolling_wheel(r.params.wheel, plain.initial.body.vel.x).omega, 0);
        check_near("with_settings: rolling, not slipping", r.initial.wheels[RR].omega * 0.33, plain.initial.body.vel.x, 1e-12);
        // settled: the springs carry exactly the weight, so the car does not start with a bounce
        const four_wheel_forces f = evaluate(r.initial, r.params, four_wheel_input{});
        check_near("with_settings: starts settled", f.fz[FL] + f.fz[FR] + f.fz[RL] + f.fz[RR],
                   r.params.chassis.mass * kGravity, 1e-6);
        check_near("with_settings: spring at its static load", f.fz[FL], r.params.static_load(FL), 1e-6);
    }

    return finish("registry");
}
