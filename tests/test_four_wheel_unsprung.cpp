#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include "runner/scenario_file.hpp"
#include "tests/check.hpp"
#include "tests/telemetry_helpers.hpp"
#include "vehicle/four_wheel_telemetry.hpp"
#include "vehicle/quarter_car.hpp"

using namespace rallysim;

// The four-wheel car with a wheel mass and a tyre spring at each corner (vertical = unsprung). Each check is
// against something that does not share its code: the car at rest, the standalone quarter-car, a closed form for
// the roll, and the springs model in the limit of a light wheel on a stiff tyre.

namespace
{
    four_wheel_params unsprung_car()
    {
        four_wheel_params p;
        p.vertical = vertical_model::unsprung;
        return p;
    }

    bool identical(const four_wheel_state &a, const four_wheel_state &b)
    {
        if (!::identical(a.body, b.body)) // check.hpp's, for the rigid body
            return false;
        for (std::size_t i = 0; i < kWheels; ++i)
            if (a.wheels[i].omega != b.wheels[i].omega || a.wheels[i].slip_ratio != b.wheels[i].slip_ratio ||
                a.wheels[i].lateral_slip != b.wheels[i].lateral_slip || a.corners[i].z != b.corners[i].z ||
                a.corners[i].vz != b.corners[i].vz)
                return false;
        return true;
    }

    // A scenario file's run on another vertical model, from a level start at `speed`, at time step dt.
    four_wheel_run in_mode(const char *name, const four_wheel_params &car, Real speed, Real dt)
    {
        four_wheel_run r = four_wheel_scenario(name);
        const Real duration = r.steps * r.dt;
        r.params = car;
        r.initial = four_wheel_initial(car, speed);
        r.dt = dt;
        r.steps = std::lround(duration / dt);
        return r;
    }
}

int main()
{
    const Real dt = 0.001, g = kGravity;
    const four_wheel_input none{};

    // at rest: level at cg_height, each tyre carrying its share of the whole weight, each strut that less its
    // wheel's weight, and nothing moves
    {
        const four_wheel_params p = unsprung_car();
        four_wheel_state s = four_wheel_initial(p, 0.0);
        const four_wheel_forces f = evaluate(s, p, none);
        Real total = 0;
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            check_near("rest: tyre load = share of the whole weight", f.fz[i], p.static_load(i), 1e-6);
            check_near("rest: strut force = tyre load - wheel weight", f.corner[i].strut, p.static_load(i) - 40 * g, 1e-6);
            check_near("rest: wheel centre at R - Fz / kt", s.corners[i].z, p.wheel.radius - p.static_load(i) / 250000, 1e-12);
            total += f.fz[i];
        }
        check_near("rest: the four tyres carry the weight", total, p.chassis.mass * g, 1e-6);
        check_near("rest: sprung mass = mass - four wheels", p.sprung_mass(), 1200 - 160, 1e-12);

        for (int i = 0; i < 10000; ++i)
            step(s, p, none, dt);
        const four_wheel_forces after = evaluate(s, p, none);
        std::printf("rest after 10 s: z %.12f, pitch %.3g, roll %.3g, x %.3g\n", s.body.pos.z, pitch_angle(s.body.q),
                    roll_angle(s.body.q), s.body.pos.x);
        check_near("rest: still at cg_height after 10 s", s.body.pos.z, p.cg_height, 1e-9);
        check_near("rest: no pitch", pitch_angle(s.body.q), 0.0, 1e-9);
        check_near("rest: no roll", roll_angle(s.body.q), 0.0, 1e-9);
        check_true("rest: not moved along the ground", std::fabs(s.body.pos.x) < 1e-9 && std::fabs(s.body.pos.y) < 1e-9);
        for (std::size_t i = 0; i < kWheels; ++i)
            check_near("rest: loads unchanged after 10 s", after.fz[i], p.static_load(i), 1e-6);
    }

    // a car with its CG midway between the axles, moving only up and down, is four quarter-cars: the body's
    // quarter of the sprung mass (260 kg) on a 40 kg wheel, the standalone model's defaults. Body lifted 10 mm,
    // then wheels lifted 5 mm: the car's body and wheels must follow vehicle/quarter_car.hpp stepped alongside.
    {
        four_wheel_params p = unsprung_car();
        p.lf = p.lr = 1.25;
        const quarter_car_params q; // 260 kg, 40 kg, 50 kN/m, 3800 N s/m, 250 kN/m
        check_near("quarter: a quarter of the sprung mass is the quarter-car's", p.sprung_mass() / 4, q.sprung_mass, 1e-12);
        const quarter_car_state rest = quarter_car_rest(q, 0);
        for (int which = 0; which < 2; ++which)
        {
            four_wheel_state s = four_wheel_initial(p, 0.0);
            quarter_car_state c = rest;
            if (which == 0)
            {
                s.body.pos.z += 0.01;
                c.zs += 0.01;
            }
            else
            {
                for (corner_state &w : s.corners)
                    w.z += 0.005;
                c.zu += 0.005;
            }
            const Real wheel_rest = p.wheel_rest_height(FL);
            Real worst_body = 0, worst_wheel = 0, travel = 0;
            for (int i = 0; i < 3000; ++i)
            {
                step(s, p, none, dt);
                step(c, q, 0, 0, dt);
                worst_body = std::max(worst_body, std::fabs((s.body.pos.z - p.cg_height) - (c.zs - rest.zs)));
                for (const corner_state &w : s.corners)
                    worst_wheel = std::max(worst_wheel, std::fabs((w.z - wheel_rest) - (c.zu - rest.zu)));
                travel = std::max(travel, std::fabs(c.zs - rest.zs));
            }
            std::printf("quarter-car, %s lifted: body within %.3g m, wheels within %.3g m over 3 s (body travel %.4f m)\n",
                        which == 0 ? "body" : "wheels", worst_body, worst_wheel, travel);
            check_true(which == 0 ? "quarter: body lifted, the body follows the quarter-car (1e-9 m)"
                                  : "quarter: wheels lifted, the body follows the quarter-car (1e-9 m)",
                       worst_body <= 1e-9);
            check_true(which == 0 ? "quarter: body lifted, the wheels follow the quarter-car (1e-9 m)"
                                  : "quarter: wheels lifted, the wheels follow the quarter-car (1e-9 m)",
                       worst_wheel <= 1e-9);
            check_near("quarter: the car stays level", pitch_angle(s.body.q), 0.0, 1e-12);
        }
    }

    // steady left turn at 20 m/s: the body rolls by m a_y h / (K_roll - m g h), with each corner's rate the
    // spring and the tyre in series, ks kt / (ks + kt); the tyre loads balance the roll moment and carry the weight
    {
        const four_wheel_params p = unsprung_car();
        four_wheel_run r;
        r.params = p;
        r.initial = four_wheel_initial(p, 20.0);
        r.hold_speed = constant_script<Real>(20.0);
        r.script.keys = {{0.0, four_wheel_input{0.0277, 0.0, 0.0, 0.0}}};
        r.steps = 8000;
        const four_wheel_state s = run(r);
        const body_velocity v = velocity_in_body(s.body);
        const Real m = p.chassis.mass, h = p.cg_height, t = p.track_front;
        const Real a_y = std::hypot(v.u, v.v) * v.r, roll = roll_angle(s.body.q);
        const Real series = 50000.0 * 250000.0 / (50000.0 + 250000.0);
        const Real k_roll = (series * t * t + series * t * t) / 2;
        const Real expected = m * a_y * h / (k_roll - m * g * h);
        const four_wheel_forces f = evaluate(s, p, driver_input(r, s, r.steps * r.dt));
        const Real right = f.fz[FR] + f.fz[RR], left = f.fz[FL] + f.fz[RL];
        std::printf("steady turn: a_y %.3f m/s^2, roll %.4f rad (closed form %.4f), load moment %.0f N m (m h (a_y + g roll) = %.0f)\n",
                    a_y, roll, expected, (right - left) * t / 2, m * h * (a_y + g * roll));
        check_true("roll: positive (right side down) in a left turn", roll > 0);
        check_near("roll: within 3% of m a_y h / (K_roll - m g h), spring and tyre in series", roll, expected, 0.03 * expected);
        check_near("load transfer: (right - left) t / 2 = m h (a_y + g roll), within 2%", (right - left) * t / 2,
                   m * h * (a_y + g * roll), 0.02 * m * h * (a_y + g * roll));
        check_near("load transfer: the four tyres still carry the weight", right + left, m * g, 0.002 * m * g);
    }

    // a light wheel on a stiff tyre is the springs model: 2 kg and 5 MN/m, both at 0.1 ms (the light wheel needs
    // the small step), over the lane change and the stop
    {
        four_wheel_params light = unsprung_car();
        for (corner_params *c : {&light.front_corner, &light.rear_corner})
        {
            c->unsprung_mass = 2;
            c->tyre_rate = 5e6;
        }
        const four_wheel_params springs;
        const Real h = 0.0001;
        const four_wheel_state lane_s = run(in_mode("fw_lane", springs, 15.0, h)), lane_u = run(in_mode("fw_lane", light, 15.0, h));
        const four_wheel_state brake_s = run(in_mode("fw_brake", springs, 25.0, h)), brake_u = run(in_mode("fw_brake", light, 25.0, h));
        const Real lane_gap = std::hypot(lane_u.body.pos.x - lane_s.body.pos.x, lane_u.body.pos.y - lane_s.body.pos.y);
        const Real brake_gap = std::fabs(brake_u.body.pos.x - brake_s.body.pos.x);
        std::printf("light wheel against springs: lane change ends %.3g m apart, stop %.3g m apart\n", lane_gap, brake_gap);
        check_true("reduction: the lane change ends within 0.1 mm of the springs model's", lane_gap < 1e-4);
        check_true("reduction: the stop is within 2.5 mm of the springs model's", brake_gap < 2.5e-3);
    }

    // the same light wheel on fw_circle: the tyre spring in series softens the roll (by the tyre's share, about 1%)
    // and the path stays within millimetres
    {
        four_wheel_params light = unsprung_car();
        for (corner_params *c : {&light.front_corner, &light.rear_corner})
        {
            c->unsprung_mass = 2;
            c->tyre_rate = 5e6;
        }
        const four_wheel_params springs;
        const four_wheel_run base = four_wheel_scenario("fw_circle");
        const Real speed = base.initial.body.vel.x;
        const four_wheel_state cs = run(in_mode("fw_circle", springs, speed, 0.0001));
        const four_wheel_state cu = run(in_mode("fw_circle", light, speed, 0.0001));
        const Real gap = std::hypot(cu.body.pos.x - cs.body.pos.x, cu.body.pos.y - cs.body.pos.y);
        const Real roll_s = roll_angle(cs.body.q), roll_u = roll_angle(cu.body.q);
        std::printf("light wheel on fw_circle: ends %.3g m apart, roll %.5f rad against %.5f (ratio %.4f)\n", gap, roll_u,
                    roll_s, roll_u / roll_s);
        check_true("reduction: the circle ends within 0.5 mm of the springs model's", gap < 5e-4);
        check_true("reduction: the roll is 0 to 3% more than the springs model's", roll_u > roll_s && roll_u < 1.03 * roll_s);
    }

    // a free drop does not reach the droop stop: body and wheel fall together
    {
        const four_wheel_params p = unsprung_car();
        four_wheel_state s = four_wheel_initial(p, 0.0);
        s.body.pos.z += 0.3;
        for (corner_state &w : s.corners)
            w.z += 0.3;
        Real most = 0;
        for (int i = 0; i < 4000; ++i)
        {
            const four_wheel_forces f = evaluate(s, p, none);
            for (std::size_t k = 0; k < kWheels; ++k)
                most = std::max(most, p.strut_static_deflection(k) - f.corner[k].deflection);
            step(s, p, none, dt);
        }
        std::printf("drop 0.3 m: the struts extend at most %.4f m of the %.2f m allowed\n", most, p.front_corner.droop);
        check_true("drop: the droop stop is never reached (extension below the droop)", most < p.front_corner.droop);
    }

    // a lifted wheel hangs on its droop stop. The springs test's wheel-lift car (high CG, very stiff front), with
    // a softer rear (30 kN/m: the tyre springs in series even out the roll stiffness, and with 50 kN/m the inner
    // front stays down with 60 N on it) and only 10 mm of droop at the front so the stop is reached: the inner
    // front tyre leaves the ground and its strut ends extended by the droop plus the wheel's weight on the stop,
    // m_u g / stop_rate
    {
        four_wheel_params q = unsprung_car();
        q.set_tyre_curve(tyre_curve::linear);
        q.front_corner.spring_rate = 300000;
        q.front_corner.droop = 0.01;
        q.rear_corner.spring_rate = 30000;
        q.cg_height = 0.85;
        four_wheel_run r;
        r.params = q;
        r.initial = four_wheel_initial(q, 18.0);
        r.hold_speed = constant_script<Real>(18.0);
        r.steps = 6000;
        r.script.keys = {{0.0, four_wheel_input{}}, {0.5, four_wheel_input{}},
                         {1.5, four_wheel_input{0.09, 0.0, 0.0, 0.0}}, {6.0, four_wheel_input{0.09, 0.0, 0.0, 0.0}}};
        four_wheel_state s = r.initial;
        Real lowest = 1e9, lifted_for = 0, worst_roll = 0, most_extended = 0;
        bool others_down = true;
        for (long i = 0; i < r.steps; ++i)
        {
            const four_wheel_input in = driver_input(r, s, i * r.dt);
            const four_wheel_forces f = evaluate(s, q, in);
            for (std::size_t w = 0; w < kWheels; ++w)
                lowest = std::min(lowest, f.fz[w]);
            if (f.fz[FL] == 0)
                lifted_for += r.dt;
            others_down = others_down && f.fz[FR] > 0 && f.fz[RL] > 0 && f.fz[RR] > 0;
            most_extended = std::max(most_extended, q.strut_static_deflection(FL) - f.corner[FL].deflection);
            worst_roll = std::max(worst_roll, std::fabs(roll_angle(s.body.q)));
            step(s, q, in, r.dt);
        }
        const four_wheel_forces end = evaluate(s, q, driver_input(r, s, r.steps * r.dt));
        const Real hang = 0.01 + 40 * g / 500000;
        const Real extension = q.strut_static_deflection(FL) - end.corner[FL].deflection;
        std::printf("lift: inner front off the ground for %.2f s, lowest load %.1f N, largest roll %.2f deg; strut extended "
                    "%.5f m at the end (droop + m_u g / stop_rate = %.5f m), most %.5f m, stop force %.1f N\n",
                    lifted_for, lowest, worst_roll * 180 / kPi, extension, hang, most_extended, end.corner[FL].stop);
        check_true("lift: the inner front tyre leaves the ground and stays up", lifted_for > 2.0);
        check_true("lift: no tyre load is ever negative", lowest >= 0);
        check_true("lift: the other three tyres stay on the ground", others_down);
        check_true("lift: the car stays upright", worst_roll < 5 * kPi / 180);
        check_near("lift: the hanging wheel rests on its droop stop, at droop + m_u g / stop_rate", extension, hang, 1e-6);
        check_near("lift: the stop carries the wheel's weight", end.corner[FL].stop + end.corner[FL].spring, -40 * g, 0.02);
        {
            const std::vector<std::string> names = column_names(four_wheel_columns());
            const std::vector<Real> row = four_wheel_row(r.steps, r.dt, s, q, driver_input(r, s, r.steps * r.dt), energy_books(s, q));
            check_column(names, row, "wheel.FL.stopF", -40 * g, 0.02);
            check_column(names, row, "wheel.FL.Fz", 0.0);
        }
    }

    // dropped from 0.3 m: in the air no tyre carries anything; it lands and settles level at its ride height
    {
        const four_wheel_params p = unsprung_car();
        four_wheel_state s = four_wheel_initial(p, 0.0);
        s.body.pos.z += 0.3;
        for (corner_state &w : s.corners)
            w.z += 0.3;
        int air = 0;
        Real lowest_load = 1e9, peak = 0, lowest_cg = 1e9;
        for (int i = 0; i < 4000; ++i)
        {
            const four_wheel_forces f = evaluate(s, p, none);
            bool off = true;
            for (std::size_t k = 0; k < kWheels; ++k)
            {
                off = off && f.fz[k] == 0;
                lowest_load = std::min(lowest_load, f.fz[k]);
                peak = std::max(peak, f.fz[k]);
            }
            air += off;
            step(s, p, none, dt);
            lowest_cg = std::min(lowest_cg, s.body.pos.z);
        }
        const four_wheel_forces f = evaluate(s, p, none);
        std::printf("drop 0.3 m: %d steps with every tyre off the ground, peak load %.0f N, lowest CG %.4f m, ends at %.9f m\n",
                    air, peak, lowest_cg, s.body.pos.z);
        check_true("drop: every tyre is off the ground at first (over 200 steps)", air > 200);
        check_true("drop: no tyre load is ever negative", lowest_load >= 0);
        check_near("drop: settles at its ride height", s.body.pos.z, p.cg_height, 1e-4);
        check_near("drop: settles level", pitch_angle(s.body.q), 0.0, 1e-6);
        for (std::size_t k = 0; k < kWheels; ++k)
            check_near("drop: each tyre back at its static load", f.fz[k], p.static_load(k), 1.0);
    }

    // a right run mirrors a left run; the same run gives the same bits
    {
        const four_wheel_params p = unsprung_car();
        const four_wheel_run left = in_mode("fw_lane", p, 15.0, dt);
        four_wheel_run right = left;
        for (auto &key : right.script.keys)
            key.value.steer = -key.value.steer;
        const four_wheel_state a = run(left), b = run(right);
        check_near("mirror: y", a.body.pos.y, -b.body.pos.y, 1e-9);
        check_near("mirror: roll", roll_angle(a.body.q), -roll_angle(b.body.q), 1e-9);
        check_near("mirror: height", a.body.pos.z, b.body.pos.z, 1e-9);
        check_near("mirror: left wheels become right wheels", a.corners[FL].z, b.corners[FR].z, 1e-9);
        check_true("determinism: two runs give identical bits, wheels' heights included", identical(a, run(left)));
    }

    // the finite guard names a wheel's height and vertical speed
    {
        four_wheel_state s = four_wheel_initial(unsprung_car(), 0.0);
        check_true("finite: a good state has no bad field", non_finite_field(s) == nullptr);
        s.corners[RL].z = std::numeric_limits<Real>::quiet_NaN();
        check_true("finite: a wheel's height is named", std::string(non_finite_field(s)) == "wheel RL z");
        s.corners[RL].z = 0.3;
        s.corners[FR].vz = std::numeric_limits<Real>::infinity();
        check_true("finite: a wheel's vertical speed is named", std::string(non_finite_field(s)) == "wheel FR vz");
    }

    // undamped heave: the body lifted 5 mm, no damping anywhere, and the frequency read off the upward zero
    // crossings of its vertical speed. Against the two-mass closed form (k, kt, a quarter of the sprung mass, one
    // wheel) for a car with its CG midway between the axles, where heave is four quarter-cars; the default car,
    // whose axles differ, is slightly lower and is held to its own measured value (2.0204 Hz)
    {
        auto heave_hz = [&](four_wheel_params p)
        {
            for (corner_params *c : {&p.front_corner, &p.rear_corner})
            {
                c->damper_rate = 0;
                c->tyre_damping = 0;
            }
            four_wheel_state s = four_wheel_initial(p, 0.0);
            s.body.pos.z += 0.005;
            Real first = -1, last = -1, prev = s.body.vel.z, prev_t = 0;
            int crossings = 0;
            for (int i = 1; i <= 5000; ++i)
            {
                step(s, p, none, 0.0005);
                const Real t = i * 0.0005, v = s.body.vel.z;
                if (prev < 0 && v >= 0)
                {
                    const Real tc = prev_t + (t - prev_t) * (-prev) / (v - prev); // the crossing, interpolated
                    first = crossings == 0 ? tc : first;
                    last = tc;
                    ++crossings;
                }
                prev = v;
                prev_t = t;
            }
            return crossings > 2 ? (crossings - 1) / (last - first) : Real(0);
        };
        four_wheel_params mid = unsprung_car();
        mid.lf = mid.lr = 1.25;
        const Real ks = 50000, kt = 250000, ms = (mid.chassis.mass - 160) / 4, mu = 40;
        const Real b = ms * (ks + kt) + mu * ks, root = std::sqrt(b * b - 4 * ms * mu * ks * kt);
        const Real expected = std::sqrt((b - root) / (2 * ms * mu)) / (2 * kPi);
        const Real f_mid = heave_hz(mid), f_default = heave_hz(unsprung_car());
        std::printf("heave: %.4f Hz (closed form %.4f Hz) with the CG midway, %.4f Hz for the default car\n", f_mid,
                    expected, f_default);
        check_near("heave: the body mode matches the two-mass closed form within 0.3%", f_mid, expected, 0.003 * expected);
        check_near("heave: the default car's body mode, 2.0204 Hz", f_default, 2.0204, 5e-5);
    }

    // telemetry: at rest, the three wheel columns against the closed forms; the same columns are 0 in the other
    // modes; line 1 names the mode
    {
        const std::vector<std::string> names = column_names(four_wheel_columns());
        const four_wheel_params p = unsprung_car();
        const four_wheel_state s = four_wheel_initial(p, 0.0);
        const std::vector<Real> row = four_wheel_row(0, dt, s, p, none, energy_books(s, p));
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            const std::string w = std::string("wheel.") + wheel_label(i) + ".";
            check_column(names, row, w + "tyreDefl", p.static_load(i) / p.corner(i).tyre_rate, 1e-12);
            check_column(names, row, w + "wheelZ", p.wheel_rest_height(i), 1e-12);
            check_column(names, row, w + "stopF", 0.0);
            check_column(names, row, w + "Fz", p.static_load(i), 1e-9);
        }
        for (vertical_model m : {vertical_model::planar, vertical_model::springs})
        {
            four_wheel_params o;
            o.vertical = m;
            const four_wheel_state so = four_wheel_initial(o, 10.0);
            const std::vector<Real> ro = four_wheel_row(0, dt, so, o, none, energy_books(so, o));
            for (std::size_t i = 0; i < kWheels; ++i)
                for (const char *c : {"tyreDefl", "wheelZ", "stopF"})
                    check_column(names, ro, std::string("wheel.") + wheel_label(i) + "." + c, 0.0);
        }
        auto meta_value = [](const four_wheel_params &q, const char *key)
        {
            for (const auto &kv : four_wheel_meta(q))
                if (kv.first == key)
                    return kv.second;
            return Real(-1);
        };
        const vertical_model modes[3] = {vertical_model::planar, vertical_model::springs, vertical_model::unsprung};
        for (int k = 0; k < 3; ++k)
        {
            four_wheel_params o;
            o.vertical = modes[k];
            check_near("meta: vertical", meta_value(o, "vertical"), k, 0);
            check_near("meta: springs is 1 for both sprung modes", meta_value(o, "springs"), k == 0 ? 0 : 1, 0);
        }
    }

    // the step number against hand values (default car, 1 ms and 2 ms), then each term on its own
    {
        const four_wheel_params p = unsprung_car();
        const Real f1 = corner_step_number(p, FL, 0.001), r1 = corner_step_number(p, RL, 0.001);
        const Real f2 = corner_step_number(p, FL, 0.002);
        std::printf("step number, default car: front %.4f, rear %.4f at 1 ms; front %.4f at 2 ms\n", f1, r1, f2);
        check_near("step number: front at 1 ms", f1, 0.2369, 1e-4);
        check_near("step number: rear at 1 ms", r1, 0.2358, 1e-4);
        check_near("step number: front at 2 ms", f2, 0.5175, 1e-4);

        four_wheel_params q = p;
        q.front_corner.rebound_ratio = 3;
        check_near("step number: rebound ratio 3 adds 2 dt (3 - 1) c / mr", corner_step_number(q, FL, 0.001) - f1, 0.43027, 1e-4);
        q.front_corner.rebound_ratio = 0.5;
        check_true("step number: a rebound ratio below 1 changes nothing", corner_step_number(q, FL, 0.001) == f1);

        four_wheel_params b = p;
        b.front_corner.bump_stop_rate = 1e6;
        const Real with_bump = corner_step_number(b, FL, 0.001);
        check_near("step number: a stiffer bump stop adds dt^2 dk / mr", with_bump - f1, 0.014153, 1e-5);
        four_wheel_params s = p;
        s.front_corner.stop_rate = 1e6;
        check_true("step number: the larger of the two stops counts", corner_step_number(s, FL, 0.001) == with_bump);

        four_wheel_params o4 = unsprung_car();
        for (corner_params *c : {&o4.front_corner, &o4.rear_corner})
        {
            c->unsprung_mass = 2;
            c->tyre_rate = 5e6;
        }
        const Real o4_front = corner_step_number(o4, FL, 0.001);
        std::printf("step number, 2 kg wheels on 5 MN/m tyres: front %.4f at 1 ms\n", o4_front);
        check_near("step number: the light-wheel set", o4_front, 6.6010, 1e-3);
        check_true("step number: the largest dt gives the limit",
                   std::fabs(corner_step_number(o4, FL, corner_step_largest_dt(o4, FL, 2.0)) - 2.0) < 1e-12);
    }

    // the claim behind the number, on a linear two-mass step written here (body on a strut over a wheel on a tyre,
    // the car's scheme: both velocities from the start-of-step forces, then positions). At a screening number of
    // 3.9 the motion must not grow; past the coupled limit it does.
    {
        struct two_mass
        {
            Real Mb, mu, ks, kt, cs, ct;
        };
        // largest |z| of body and wheel over steps [from, to) of a run of `steps` steps at step h
        auto run_two_mass = [](const two_mass &m, Real h, int steps, int window, Real &early, Real &late, bool &finite)
        {
            Real zb = 0.01, zu = 0, vb = 0, vu = 0;
            early = late = 0;
            finite = true;
            for (int i = 0; i < steps; ++i)
            {
                const Real S = m.ks * (zu - zb) + m.cs * (vu - vb);
                vb += h * S / m.Mb;
                vu += h * (-S - m.kt * zu - m.ct * vu) / m.mu;
                zb += h * vb;
                zu += h * vu;
                const Real big = std::max(std::fabs(zb), std::fabs(zu));
                if (!std::isfinite(big))
                {
                    finite = false;
                    early = late = std::numeric_limits<Real>::infinity();
                    return;
                }
                if (i < window)
                    early = std::max(early, big);
                if (i >= steps - window)
                    late = std::max(late, big);
            }
        };
        const two_mass cases[] = {{260, 40, 5e4, 2.5e5, 3800, 0}, {260, 2, 5e4, 5e6, 3800, 0},   {30, 40, 5e4, 2.5e5, 11400, 0},
                                  {600, 2, 6e5, 1e5, 12000, 0},   {30, 80, 2e5, 5e6, 3800, 0},   {100, 10, 2e5, 1e6, 0, 2000}};
        for (const two_mass &m : cases)
        {
            const Real mr = m.mu * m.Mb / (m.mu + m.Mb);
            const Real a = m.ks / mr + m.kt / m.mu, b = 2 * (m.cs / mr + m.ct / m.mu);
            const Real h = (-b + std::sqrt(b * b + 4 * a * 3.9)) / (2 * a);
            Real early, late;
            bool finite;
            run_two_mass(m, h, 20000, 2000, early, late, finite);
            std::printf("two-mass (%g, %g, %g, %g, %g, %g) at Sr 3.9, h %.4g s: late / early = %.3g\n", m.Mb, m.mu, m.ks, m.kt,
                        m.cs, m.ct, h, late / early);
            check_true("claim: a case at screening number 3.9 does not grow", finite && late <= early);
        }
        // control: the same code past the limit must show growth
        Real early, late;
        bool finite;
        run_two_mass(cases[1], 0.00075, 2000, 2000, early, late, finite);
        std::printf("control (260, 2, ...) at h 0.75 ms: %s, largest |z| %.3g (start 0.01)\n", finite ? "finite" : "non-finite", late);
        check_true("claim: the control grows past 1e6 times its start", !finite || late > 1e6 * 0.01);
    }

    // the car itself: the light-wheel set on fw_lane at 0.7 ms (simple number 3.90) keeps the front-left tyre
    // load within 20% of its static load at every step; at 0.8 ms (4.66) the range is printed only
    {
        four_wheel_params o4 = unsprung_car();
        for (corner_params *c : {&o4.front_corner, &o4.rear_corner})
        {
            c->unsprung_mass = 2;
            c->tyre_rate = 5e6;
        }
        for (Real h : {0.0007, 0.0008})
        {
            const four_wheel_run r = in_mode("fw_lane", o4, 15.0, h);
            four_wheel_state s = r.initial;
            Real lo = 1e30, hi = -1e30;
            for (long i = 0; i < r.steps; ++i)
            {
                const four_wheel_input in = driver_input(r, s, i * r.dt);
                const Real fz = evaluate(s, o4, in).fz[FL];
                lo = std::min(lo, fz);
                hi = std::max(hi, fz);
                step(s, o4, in, r.dt);
            }
            std::printf("light wheels on fw_lane at %.1f ms: FL load %.0f to %.0f N (static %.0f N)\n", h * 1000, lo, hi,
                        o4.static_load(FL));
            if (h < 0.00075)
                check_true("car: at 0.7 ms the FL load stays within 20% of its static load",
                           lo >= 0.8 * o4.static_load(FL) && hi <= 1.2 * o4.static_load(FL));
        }
    }

    return finish("four_wheel_unsprung");
}
