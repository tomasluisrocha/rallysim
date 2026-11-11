#include <algorithm>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>
#include "check.hpp"
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

    // apply() refuses the value with exactly `message`, and leaves the car as it was.
    bool refused(const std::string &name, const std::string &value, const std::string &message)
    {
        four_wheel_params p;
        const Real before = find_param(name)->field(p);
        try
        {
            apply(p, name, value);
        }
        catch (const std::invalid_argument &e)
        {
            return message == e.what() && find_param(name)->field(p) == before;
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
        // every parameter is on line 1, once, so a file records the whole car that made it (--set included)
        const telemetry_meta meta = four_wheel_meta(p);
        for (const param_entry &e : four_wheel_registry())
        {
            int times = 0;
            for (const auto &kv : meta)
                times += kv.first == e.name ? 1 : 0;
            check_true(e.name, times == 1);
        }
        for (std::size_t i = 0; i < meta.size(); ++i)
            for (std::size_t j = i + 1; j < meta.size(); ++j)
                check_true("line 1: no key twice", meta[i].first != meta[j].first);
    }
    {
        // when a parameter is used
        four_wheel_params p;
        check_true("used: a spring on the springs car", param_used(*find_param("spring_front"), p));
        check_true("used: no transfer share on the springs car", !param_used(*find_param("front_transfer_share"), p));
        check_true("used: no peak_slide shape on a tmeasy tyre", !param_used(*find_param("peak_slip_front"), p));
        p.vertical = vertical_model::planar;
        p.front_tyre.curve = tyre_curve::peak_slide;
        check_true("used: no spring on the planar car", !param_used(*find_param("damper_rear"), p));
        check_true("used: the transfer share on the planar car", param_used(*find_param("front_transfer_share"), p));
        check_true("used: the front shape on a front peak_slide", param_used(*find_param("sliding_ratio_front"), p));
        check_true("used: not the rear shape", !param_used(*find_param("sliding_ratio_rear"), p));
        check_true("used: grip always", param_used(*find_param("mu_rear"), p));
        p.vertical = vertical_model::unsprung;
        check_true("used: a spring on the unsprung car", param_used(*find_param("spring_front"), p));
        check_true("used: the wheel mass on the unsprung car", param_used(*find_param("unsprung_mass_rear"), p));
        check_true("used: no transfer share on the unsprung car", !param_used(*find_param("front_transfer_share"), p));
        p.vertical = vertical_model::springs;
        for (const char *n : {"unsprung_mass_front", "tyre_rate_rear", "tyre_damping_front", "droop_rear"})
            check_true("used: no wheel number on the springs car", !param_used(*find_param(n), p));
    }

    {
        // the wheel's numbers: the corner defaults, settable per axle, ranges as designed
        four_wheel_params p;
        check_near("wheel: unsprung_mass_front default", find_param("unsprung_mass_front")->field(p), 40, 0);
        check_near("wheel: tyre_rate_rear default", find_param("tyre_rate_rear")->field(p), 250000, 0);
        check_near("wheel: droop_front default", find_param("droop_front")->field(p), 0.10, 0);
        apply(p, "tyre_rate_front", "300 kN/m");
        apply(p, "droop_rear", "80 mm");
        apply(p, "unsprung_mass_rear", "35");
        apply(p, "tyre_damping_front", "0");
        check_near("wheel: tyre_rate_front set", p.front_corner.tyre_rate, 300000, 1e-9);
        check_near("wheel: tyre_rate_rear untouched", p.rear_corner.tyre_rate, 250000, 0);
        check_near("wheel: droop_rear set", p.rear_corner.droop, 0.08, 1e-15);
        check_near("wheel: unsprung_mass_rear set", p.rear_corner.unsprung_mass, 35, 0);
        check_true("wheel: unsprung_mass 0 refused", throws("unsprung_mass_front", "0"));
        check_true("wheel: tyre_rate 0 refused", throws("tyre_rate_rear", "0"));
        check_true("wheel: droop 0 refused", throws("droop_front", "0"));
        check_true("wheel: negative tyre_damping refused", throws("tyre_damping_rear", "-1"));
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

    // ---- ranges ---------------------------------------------------------------------------------------------
    {
        four_wheel_params p;
        for (const param_entry &e : table)
        {
            check_true(e.name, e.range != param_range::any); // every number so far has one
            check_true(e.name, range_error(e, e.field(p)).empty());
        }
    }
    check_true("range: mass = 0", refused("mass", "0", "parameter mass: must be above 0"));
    check_true("range: a negative spring", refused("spring_front", "-1", "parameter spring_front: must be above 0"));
    check_true("range: a negative damper", refused("damper_rear", "-1", "parameter damper_rear: must be 0 or more"));
    check_true("range: a bias above 1",
               refused("brake_bias_front", "1.5", "parameter brake_bias_front: must be within 0 to 1"));
    check_true("range: a peak past 3", refused("peak_slip_front", "4", "parameter peak_slip_front: must be within 1.5 to 3"));
    check_true("range: the ends are inside", !throws("drive_split_front", "100 %") && !throws("mu_front", "0") &&
                                                 !throws("peak_slip_rear", "1.5") && !throws("peak_slip_rear", "3"));

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

    return finish("registry");
}
