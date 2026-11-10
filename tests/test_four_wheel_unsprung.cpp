#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include "runner/scenario_file.hpp"
#include "tests/check.hpp"
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

    // a light wheel on a stiff tyre is the springs model: 2 kg and 5 MN/m, both at 0.2 ms (the light wheel needs
    // the small step), over the lane change and the stop
    {
        four_wheel_params light = unsprung_car();
        for (corner_params *c : {&light.front_corner, &light.rear_corner})
        {
            c->unsprung_mass = 2;
            c->tyre_rate = 5e6;
        }
        const four_wheel_params springs;
        const Real h = 0.0002;
        const four_wheel_state lane_s = run(in_mode("fw_lane", springs, 15.0, h)), lane_u = run(in_mode("fw_lane", light, 15.0, h));
        const four_wheel_state brake_s = run(in_mode("fw_brake", springs, 25.0, h)), brake_u = run(in_mode("fw_brake", light, 25.0, h));
        const Real lane_gap = std::hypot(lane_u.body.pos.x - lane_s.body.pos.x, lane_u.body.pos.y - lane_s.body.pos.y);
        const Real brake_gap = std::fabs(brake_u.body.pos.x - brake_s.body.pos.x);
        std::printf("light wheel against springs: lane change ends %.3g m apart, stop %.3g m apart\n", lane_gap, brake_gap);
        check_true("reduction: the lane change ends within 1 mm of the springs model's", lane_gap < 1e-3);
        check_true("reduction: the stop is within 5 mm of the springs model's", brake_gap < 5e-3);
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
        check_near("lift: the hanging wheel rests on its droop stop, at droop + m_u g / stop_rate", extension, hang, 1e-4);
        check_near("lift: the stop carries the wheel's weight", end.corner[FL].stop + end.corner[FL].spring, -40 * g, 2.0);
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

    return finish("four_wheel_unsprung");
}
