#include <algorithm>
#include <cmath>
#include <cstdio>
#include "scenarios/four_wheel_lane_change.hpp"
#include "tests/check.hpp"
#include "tests/telemetry_helpers.hpp"

using namespace rallysim;

// The four-wheel car piece by piece, with the planar vertical model (the car flat on the ground plane, loads
// from the quasi-static transfer): where the wheels are, how fast each contact patch moves, the loads, how the
// tyre forces reach the body, and simple runs with closed-form answers. The springs model is tested in
// test_four_wheel_springs.cpp, the comparison with the single-track (the phase 4 gate) in
// test_four_wheel_behaviour.cpp.

static four_wheel_params planar_car()
{
    four_wheel_params p;
    p.vertical = vertical_model::planar;
    return p;
}

// A car at heading `yaw`, velocity (u, v) in its own frame, yaw rate r, every wheel rolling at u.
static four_wheel_state moving(const four_wheel_params &p, Real yaw, Real u, Real v, Real r)
{
    four_wheel_state s = four_wheel_initial(p, u);
    s.body.q = from_axis_angle(vec3{0, 0, 1}, yaw);
    s.body.vel = to_world(s.body.q, vec3{u, v, 0});
    s.body.omega = vec3{0, 0, r};
    return s;
}

static bool identical(const four_wheel_state &a, const four_wheel_state &b)
{
    if (!identical(a.body, b.body))
        return false;
    for (std::size_t i = 0; i < kWheels; ++i)
        if (a.wheels[i].omega != b.wheels[i].omega || a.wheels[i].slip_ratio != b.wheels[i].slip_ratio ||
            a.wheels[i].lateral_slip != b.wheels[i].lateral_slip)
            return false;
    return true;
}

int main()
{
    const four_wheel_params p = planar_car(); // 1200 kg, lf 1.3, lr 1.2, tracks 1.5, h 0.5; tyres 30 / 35 kN/rad
    const Real weight = p.chassis.mass * kGravity;
    const Real dt = 0.001;

    // where the wheels are: x forward, y left, from the CG
    {
        check_vec("position: front left", p.wheel_position(FL), vec3{1.3, 0.75, 0});
        check_vec("position: front right", p.wheel_position(FR), vec3{1.3, -0.75, 0});
        check_vec("position: rear left", p.wheel_position(RL), vec3{-1.2, 0.75, 0});
        check_vec("position: rear right", p.wheel_position(RR), vec3{-1.2, -0.75, 0});
        check_true("front wheels use the front tyre", p.tyre(FL).cornering_stiffness == 30000 && p.tyre(RR).cornering_stiffness == 35000);
    }

    // contact patch velocity: v_body + omega x r, then into the wheel's frame
    {
        const four_wheel_state straight = moving(p, 0.7, 20.0, 0.0, 0.0);
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            const patch_velocity v = patch_velocity_of(p, straight.body, i, 0.0);
            check_true("patch: straight ahead every patch moves at (20, 0), whatever the heading",
                       std::fabs(v.vx - 20.0) < 1e-12 && std::fabs(v.vy) < 1e-12);
        }

        // turning left at r = 0.2: the inner (left) wheels go slower, the outer faster, by r t / 2; the front
        // moves left at lf r, the rear right at lr r
        const four_wheel_state turning = moving(p, 0.0, 20.0, 0.0, 0.2);
        const patch_velocity fl = patch_velocity_of(p, turning.body, FL, 0.0);
        const patch_velocity fr = patch_velocity_of(p, turning.body, FR, 0.0);
        const patch_velocity rl = patch_velocity_of(p, turning.body, RL, 0.0);
        check_near("patch: inner front wheel slower by r t / 2", fl.vx, 20.0 - 0.2 * 0.75, 1e-12);
        check_near("patch: outer front wheel faster by r t / 2", fr.vx, 20.0 + 0.2 * 0.75, 1e-12);
        check_near("patch: front moves left at lf r", fl.vy, 1.3 * 0.2, 1e-12);
        check_near("patch: rear moves right at lr r", rl.vy, -1.2 * 0.2, 1e-12);

        // a steered wheel sees the same velocity turned by -steer: a car going straight with the wheels
        // turned left has its patches moving to the right of where the wheels point
        const patch_velocity steered = patch_velocity_of(p, straight.body, FL, 0.1);
        check_near("patch: steered wheel, along it", steered.vx, 20.0 * std::cos(0.1), 1e-12);
        check_near("patch: steered wheel, across it (to its right)", steered.vy, -20.0 * std::sin(0.1), 1e-12);
        check_near("patch: the rear wheels are not steered", patch_velocity_of(p, straight.body, RL, 0.1).vy, 0.0, 1e-12);
    }

    // loads: static split, front to rear with a_x, side to side with a_y; always the weight in total
    {
        auto total = [](const std::array<Real, kWheels> &f) { return f[FL] + f[FR] + f[RL] + f[RR]; };
        const auto rest = four_wheel_loads(p, 0.0, 0.0);
        check_near("loads: front wheel at rest = W lr / 2L", rest[FL], weight * 1.2 / 2.5 / 2, 1e-9);
        check_near("loads: rear wheel at rest = W lf / 2L", rest[RR], weight * 1.3 / 2.5 / 2, 1e-9);
        check_near("loads: at rest they add up to the weight", total(rest), weight, 1e-9);

        // left turn at a_y = 4: roll moment m a_y h = 2400 N m, half per axle, over a 1.5 m track: 800 N per axle
        const auto left = four_wheel_loads(p, 0.0, 4.0);
        check_near("loads: left turn, front right gains 800 N", left[FR] - rest[FR], 800.0, 1e-9);
        check_near("loads: left turn, rear left loses 800 N", left[RL] - rest[RL], -800.0, 1e-9);
        check_near("loads: left turn, total kept", total(left), weight, 1e-9);

        // braking moves m a_x h / L to the front, half per wheel
        const auto brake = four_wheel_loads(p, -5.0, 0.0);
        check_near("loads: braking, each front wheel gains m a_x h / 2L", brake[FL] - rest[FL], 1200.0 * 5.0 * 0.5 / 2.5 / 2, 1e-9);

        // a narrower front track moves more load across the front for the same share of the roll moment
        four_wheel_params narrow = p;
        narrow.track_front = 1.0;
        check_near("loads: narrower front track, more transfer (moment / track)",
                   four_wheel_loads(narrow, 0.0, 4.0)[FR] - rest[FR], 0.5 * 2400.0 / 1.0, 1e-9);

        // lift: never negative, total kept
        const auto lift = four_wheel_loads(p, 0.0, 25.0);
        check_true("loads: no wheel below zero", *std::min_element(lift.begin(), lift.end()) >= 0);
        check_near("loads: lift keeps the weight", total(lift), weight, 1e-9);
    }

    // force routing: each tyre force turned out of its wheel's frame and applied at its patch
    {
        const four_wheel_state s = moving(p, 0.0, 20.0, 0.0, 0.0);
        four_wheel_forces f;
        f.steer = 0.1;
        f.tyre[FL] = tyre_force{0, 1000}; // 1000 N to the left of the steered front-left wheel
        const wrench a = four_wheel_wrench(s, p, f);
        check_vec("routing: a steered wheel's side force leans back by the steer angle", a.force,
                  vec3{-1000 * std::sin(0.1), 1000 * std::cos(0.1), weight}, 1e-9);
        check_near("routing: and turns the car left about the CG",
                   a.torque.z, 1.3 * 1000 * std::cos(0.1) - 0.75 * (-1000 * std::sin(0.1)), 1e-9);

        four_wheel_forces g;
        g.tyre[RL] = tyre_force{500, 0}; // the rear-left wheel pushes forward
        check_near("routing: pushing the left side forward turns the car right", four_wheel_wrench(s, p, g).torque.z,
                   -0.75 * 500, 1e-9);
        check_near("routing: no roll or pitch torque (forces at CG height)", four_wheel_wrench(s, p, g).torque.x, 0.0, 1e-12);
    }

    // at rest with no input: nothing moves, exactly
    {
        four_wheel_state s = four_wheel_initial(p, 0.0);
        const four_wheel_state start = s;
        for (int i = 0; i < 10000; ++i)
            step(s, p, four_wheel_input{}, dt);
        check_true("rest: nothing moves in 10 s", identical(s, start));
    }

    // rolling straight with no input: no force at all, the speed and line are kept exactly
    {
        four_wheel_state s = four_wheel_initial(p, 20.0);
        for (int i = 0; i < 5000; ++i)
            step(s, p, four_wheel_input{}, dt);
        check_near("rolling: speed kept", velocity_in_body(s.body).u, 20.0, 1e-12);
        check_near("rolling: line kept", s.body.pos.y, 0.0, 1e-12);
        check_near("rolling: x = v t", s.body.pos.x, 100.0, 1e-9);
    }

    // gentle drive on the rear wheels: every wheel's spin inertia adds to the mass, a = (T/R) / (m + 4 I / R^2).
    // From 5 m/s, so the wheels stay below the speed where the power limit starts (33 rad/s, 10.5 m/s).
    {
        four_wheel_state s = four_wheel_initial(p, 5.0);
        const Real throttle = 0.1, T = throttle * p.drive_brake.max_drive_torque; // 600 N m
        for (int i = 0; i < 2000; ++i)
            step(s, p, four_wheel_input{0.0, throttle}, dt);
        const Real R = p.wheel.radius, I = p.wheel.inertia;
        const Real expected = (T / R) / (p.chassis.mass + 4 * I / (R * R));
        check_near("drive: a = (T/R) / (m + 4 I/R^2)", (velocity_in_body(s.body).u - 5.0) / 2.0, expected, 0.01 * expected);
        check_true("drive: the rear wheels slip a little, the fronts roll", s.wheels[RL].slip_ratio > 0 && std::fabs(s.wheels[FL].slip_ratio) < 1e-3);
    }

    // steering left turns the car left; a right run mirrors a left run
    {
        four_wheel_state s = four_wheel_initial(p, 15.0);
        for (int i = 0; i < 1000; ++i)
            step(s, p, four_wheel_input{0.05, 0.0}, dt);
        check_true("turn: steering left gives a positive yaw rate and moves the car left",
                   s.body.omega.z > 0 && s.body.pos.y > 0 && yaw_angle(s.body.q) > 0);

        four_wheel_run left = four_wheel_lane_change();
        left.params = p;
        left.initial = four_wheel_initial(p, 15.0);
        four_wheel_run right = left;
        for (auto &k : right.script.keys)
            k.value.steer = -k.value.steer;
        const four_wheel_state a = run(left), b = run(right);
        check_near("mirror: x", a.body.pos.x, b.body.pos.x, 1e-9);
        check_near("mirror: y", a.body.pos.y, -b.body.pos.y, 1e-9);
        check_near("mirror: yaw rate", a.body.omega.z, -b.body.omega.z, 1e-9);
        check_near("mirror: left and right wheels swap", a.wheels[FL].omega, b.wheels[FR].omega, 1e-9);
        check_near("plane: stays on the ground", a.body.pos.z, 0.0, 1e-9);
        check_true("determinism: two runs give identical bits", identical(a, run(left)));
    }

    // friction: a hard steer at speed, and no tyre force ever exceeds that tyre's grip
    {
        four_wheel_state s = four_wheel_initial(p, 20.0);
        bool within = true, reached = false;
        for (int i = 0; i < 3000; ++i)
        {
            const four_wheel_input in{0.4, 0.05};
            const four_wheel_forces f = evaluate(s, p, in);
            for (std::size_t w = 0; w < kWheels; ++w)
            {
                const Real size = std::hypot(f.tyre[w].fx, f.tyre[w].fy), grip = tyre_grip(p.tyre(w), f.fz[w]);
                within = within && size <= grip * (1 + 1e-12);
                // within 1% of the grip: the car's tmeasy tyre passes through its peak on its way to sliding,
                // and a 1 ms sample seldom lands exactly on it
                reached = reached || size >= grip * 0.99;
            }
            step(s, p, in, dt);
        }
        check_true("friction: some tyre reaches its grip", reached);
        check_true("friction: no tyre force ever exceeds its grip", within);
    }

    // telemetry: one value per column, under the right name
    {
        const std::vector<std::string> names = four_wheel_columns(), units = four_wheel_units();
        const four_wheel_state s = moving(p, 0.3, 12.0, 0.4, 0.25);
        const four_wheel_input in{0.05, 0.2, 0.1, 0.3};
        const energy_books books(s, p); // no work done yet: the residual is zero
        const std::vector<Real> row = four_wheel_row(7, dt, s, p, in, books);
        check_true("telemetry: 18 + 4 x 12 + 11 energy columns, units and values",
                   names.size() == 77 && units.size() == 77 && row.size() == 77);
        for (std::size_t i = 0; i < names.size(); ++i)
            for (std::size_t j = i + 1; j < names.size(); ++j)
                check_true("telemetry: column names are unique", names[i] != names[j]);
        const four_wheel_forces f = evaluate(s, p, in);
        check_column(names, row, "wheel.FR.Fz", f.fz[FR]);
        check_column(names, row, "wheel.RL.omega", s.wheels[RL].omega);
        check_column(names, row, "throttle", 0.2);
        check_column(names, row, "brake", 0.1);
        check_column(names, row, "handbrake", 0.3);
        check_column(names, row, "wheel.RL.driveTorque", f.torque[RL].drive);
        check_column(names, row, "wheel.RR.brakeTorque", f.torque[RR].brake);
        check_column(names, row, "wheel.FL.driveTorque", 0.0); // rear drive
        check_column(names, row, "yaw_rate", 0.25, 1e-12);
        check_column(names, row, "beta", std::atan2(0.4, 12.0), 1e-12);
        check_column(names, row, "energy.total", energy_of(s, p).total());
        check_column(names, row, "energy.translation", 0.5 * p.chassis.mass * (12.0 * 12.0 + 0.4 * 0.4), 1e-9);
        check_column(names, row, "energy.residual", 0.0);
        for (const char *needed : {"pos.x", "pos.y", "yaw", "steer", "speed", "beta"})
            check_true("telemetry: tools/topdown.py can read it", index_of(names, needed) >= 0);
    }

    return finish("four_wheel");
}
