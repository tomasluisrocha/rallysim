#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include "runner/scenario_file.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// Golden runs for the unsprung mode: fw_lane, fw_brake and fw_circle with wheel masses on tyre springs,
// and two drops. Each case is first checked against something that does not share its code (the springs model, a
// closed form), then its recorded values are compared at 1e-9. The timestep check comes last.

static const Real golden_tol = 1e-9;

// The body's position, heading, velocities, height, roll and pitch, each wheel's spin and each wheel centre's height.
struct snapshot
{
    Real x, y, yaw, u, v, r, z, roll, pitch, w_fl, w_fr, w_rl, w_rr, h_fl, h_fr, h_rl, h_rr;
};

static snapshot snapshot_of(const four_wheel_state &s)
{
    const body_velocity v = velocity_in_body(s.body);
    return {s.body.pos.x, s.body.pos.y, yaw_angle(s.body.q), v.u, v.v, v.r, s.body.pos.z,
            roll_angle(s.body.q), pitch_angle(s.body.q),
            s.wheels[FL].omega, s.wheels[FR].omega, s.wheels[RL].omega, s.wheels[RR].omega,
            s.corners[FL].z, s.corners[FR].z, s.corners[RL].z, s.corners[RR].z};
}

static bool identical(const four_wheel_state &a, const four_wheel_state &b)
{
    if (!identical(a.body, b.body))
        return false;
    for (std::size_t i = 0; i < kWheels; ++i)
        if (a.wheels[i].omega != b.wheels[i].omega || a.wheels[i].slip_ratio != b.wheels[i].slip_ratio ||
            a.wheels[i].lateral_slip != b.wheels[i].lateral_slip || a.corners[i].z != b.corners[i].z ||
            a.corners[i].vz != b.corners[i].vz)
            return false;
    return true;
}

static void check_golden(const char *name, const four_wheel_state &s, const snapshot &e)
{
    const snapshot a = snapshot_of(s);
    const Real got[] = {a.x, a.y, a.yaw, a.u, a.v, a.r, a.z, a.roll, a.pitch, a.w_fl, a.w_fr, a.w_rl, a.w_rr,
                        a.h_fl, a.h_fr, a.h_rl, a.h_rr};
    const Real want[] = {e.x, e.y, e.yaw, e.u, e.v, e.r, e.z, e.roll, e.pitch, e.w_fl, e.w_fr, e.w_rl, e.w_rr,
                         e.h_fl, e.h_fr, e.h_rl, e.h_rr};
    const char *fields[] = {"x", "y", "yaw", "u", "v", "r", "z", "roll", "pitch", "omega FL", "omega FR", "omega RL",
                            "omega RR", "wheel FL", "wheel FR", "wheel RL", "wheel RR"};
    const int before = failures;
    for (int k = 0; k < 17; ++k)
        check_near((std::string(name) + " golden " + fields[k]).c_str(), got[k], want[k], golden_tol);
    if (failures != before)
    {
        std::printf("%s (re-record only after checking the independent checks):\n    {", name);
        for (int k = 0; k < 17; ++k)
            std::printf("%.17g%s", got[k], k < 16 ? ", " : "}\n");
    }
}

// A scenario file's run on the unsprung model, from the scenario's own start speed, at its own step.
static four_wheel_run unsprung_run(const char *name)
{
    four_wheel_run r = four_wheel_scenario(name);
    r.params.vertical = vertical_model::unsprung;
    r.initial = four_wheel_initial(r.params, r.initial.body.vel.x);
    return r;
}

static four_wheel_state springs_end(const char *name) { return run(four_wheel_scenario(name)); }

static snapshot at(four_wheel_run r, Real dt)
{
    const Real duration = r.steps * r.dt;
    r.dt = dt;
    r.steps = std::lround(duration / dt);
    return snapshot_of(run(r));
}

static void test_lane()
{
    const four_wheel_run r = unsprung_run("fw_lane");
    const four_wheel_state s = run(r);
    check_true("lane: determinism: two runs give identical bits", identical(s, run(unsprung_run("fw_lane"))));
    const body_velocity v = velocity_in_body(s.body);
    check_true("lane: ends straight (yaw rate and sideways speed under 1e-3)", std::fabs(v.r) < 1e-3 && std::fabs(v.v) < 1e-3);
    check_near("lane: body level again", roll_angle(s.body.q), 0.0, 1e-3);
    const four_wheel_state sp = springs_end("fw_lane");
    check_near("lane: ends within 1 cm of the springs model, along", s.body.pos.x, sp.body.pos.x, 0.01);
    check_near("lane: and across", s.body.pos.y, sp.body.pos.y, 0.01);
    check_golden("lane", s, {104.8500104327947, 3.4005849226863183, -3.2536437337007828e-06, 14.999999999562675, 3.602192004303526e-08,
        8.0431454765871459e-10, 0.50000000000219813, -3.5893522271465528e-09, 1.5392402514694391e-11, 47.619047615935777,
        47.619047619675676, 47.619047616834187, 47.619047620574072, 0.30369887960068898, 0.30369888039472959,
        0.30275711960520735, 0.30275712039924785});
}

static void test_brake()
{
    const four_wheel_run r = unsprung_run("fw_brake");
    const four_wheel_state s = run(r);
    check_true("brake: determinism: two runs give identical bits", identical(s, run(unsprung_run("fw_brake"))));
    bool locked = true;
    for (const wheel_state &w : s.wheels)
        locked = locked && w.omega == 0;
    check_true("brake: all four wheels end locked", locked);
    check_true("brake: the car has stopped", std::fabs(velocity_in_body(s.body).u) < 0.05);
    const four_wheel_state sp = springs_end("fw_brake");
    check_near("brake: the stop is within 5 cm of the springs model's", s.body.pos.x, sp.body.pos.x, 0.05);
    check_golden("brake", s, {67.468839312976172, -6.2887901070382337e-15, 4.7618038052992166e-16, -0.010111394242939704,
        -1.0943309527778377e-16, -3.0382582196616135e-18, 0.50033668341011117, 3.836201310295748e-17,
        0.001658871056372795, 0, 0, 0, 0, 0.30360986535950213, 0.30360986535950213, 0.30291138026723691,
        0.30291138026723691});
}

// Settled turn: the body leans out by m a_y h / (K_roll - m g h), each corner's rate the spring and the tyre in series.
static void test_circle()
{
    const four_wheel_run r = unsprung_run("fw_circle");
    const four_wheel_state s = run(r);
    check_true("circle: determinism: two runs give identical bits", identical(s, run(unsprung_run("fw_circle"))));
    const four_wheel_params &p = r.params;
    const body_velocity v = velocity_in_body(s.body);
    const Real a_y = std::hypot(v.u, v.v) * v.r;
    const Real series = p.front_corner.spring_rate * p.front_corner.tyre_rate / (p.front_corner.spring_rate + p.front_corner.tyre_rate);
    const Real k_roll = series * (p.track_front * p.track_front + p.track_rear * p.track_rear) / 2;
    const Real expected = p.chassis.mass * a_y * p.cg_height / (k_roll - p.chassis.mass * kGravity * p.cg_height);
    check_near("circle: the roll is within 3% of m a_y h / (K_roll - m g h)", roll_angle(s.body.q), expected, 0.03 * expected);
    check_golden("circle", s, {9.9315768249225105, 24.397172746901845, 2.7369720611661879, 5.9909981628253632, 0.41058429485919312,
        0.47740551969389544, 0.49992916036593243, 0.019378791824241882, 0.00046190300841480184, 18.161927727973765,
        20.389984364337405, 17.881896660002951, 20.152186317487931, 0.30602484731864338, 0.30118045310136055,
        0.30527554689863945, 0.30043115268135667});
}

// A drop from `lift` above the ride height, nothing driving: the largest load on each axle's tyres and the lowest CG.
struct landing
{
    Real peak_front = 0, peak_rear = 0, lowest = 1e9;
};

static landing drop(Real lift)
{
    four_wheel_params p;
    p.vertical = vertical_model::unsprung;
    four_wheel_state s = four_wheel_initial(p, 0.0);
    s.body.pos.z += lift;
    for (corner_state &c : s.corners)
        c.z += lift;
    landing l;
    for (int i = 0; i < 4000; ++i)
    {
        const four_wheel_forces f = evaluate(s, p, four_wheel_input{});
        for (std::size_t k = 0; k < kWheels; ++k)
            (is_front(k) ? l.peak_front : l.peak_rear) = std::max(is_front(k) ? l.peak_front : l.peak_rear, f.fz[k]);
        step(s, p, four_wheel_input{}, 0.001);
        l.lowest = std::min(l.lowest, s.body.pos.z);
    }
    return l;
}

static void test_drops()
{
    const four_wheel_params p = [] { four_wheel_params q; q.vertical = vertical_model::unsprung; return q; }();
    struct golden_drop
    {
        Real lift, peak_front, peak_rear, lowest;
    };
    const golden_drop expected[2] = {{0.3, 10614.414913909215, 10673.874336907696, 0.40886937918016802},
                                    {1.0, 19522.057065924699, 19707.000938215722, 0.30221777758715512}};
    for (const golden_drop &e : expected)
    {
        const landing l = drop(e.lift);
        const std::string name = "drop " + std::to_string(e.lift).substr(0, 3) + " m";
        check_true((name + ": determinism").c_str(), [&] { const landing again = drop(e.lift); return again.peak_front == l.peak_front && again.peak_rear == l.peak_rear && again.lowest == l.lowest; }());
        check_true((name + ": the peak tyre loads are above the static loads").c_str(),
                   l.peak_front > p.static_load(FL) && l.peak_rear > p.static_load(RL));
        const int before = failures;
        check_near((name + " golden peak front").c_str(), l.peak_front, e.peak_front, golden_tol);
        check_near((name + " golden peak rear").c_str(), l.peak_rear, e.peak_rear, golden_tol);
        check_near((name + " golden lowest CG").c_str(), l.lowest, e.lowest, golden_tol);
        if (failures != before)
            std::printf("%s (re-record only after checking the independent checks):\n    {%.17g, %.17g, %.17g, %.17g}\n",
                        name.c_str(), e.lift, l.peak_front, l.peak_rear, l.lowest);
    }
}

static void test_convergence()
{
    const snapshot lane2 = at(unsprung_run("fw_lane"), 0.002), lane1 = at(unsprung_run("fw_lane"), 0.001),
                   lane05 = at(unsprung_run("fw_lane"), 0.0005);
    const Real coarse = std::hypot(lane2.x - lane1.x, lane2.y - lane1.y), fine = std::hypot(lane1.x - lane05.x, lane1.y - lane05.y);
    std::printf("convergence, unsprung lane change: end position moves %.2e m from 2 to 1 ms, %.2e m from 1 to 0.5 ms\n", coarse, fine);
    check_true("convergence: lane change, halving dt changes the end position less each time", fine < 0.7 * coarse);

    // The stop does not shrink by a steady ratio: the instant a wheel locks lands differently at each step, so
    // the check is against a fine-step reference, 0.125 ms: the error falls at each halving and is under 1 mm at 1 ms.
    const Real ref = at(unsprung_run("fw_brake"), 0.000125).x;
    Real err[3];
    const Real dts[3] = {0.002, 0.001, 0.0005};
    for (int k = 0; k < 3; ++k)
        err[k] = std::fabs(at(unsprung_run("fw_brake"), dts[k]).x - ref);
    std::printf("convergence, unsprung braking: stop %.2e m from the 0.125 ms run at 2 ms, %.2e m at 1 ms, %.2e m at 0.5 ms\n",
                err[0], err[1], err[2]);
    check_true("convergence: braking, the stop's error against the fine run falls at each halving", err[1] < err[0] && err[2] < err[1]);
    check_true("convergence: braking, under 1 mm from the fine run at 1 ms", err[1] < 1e-3);
}

int main()
{
    test_lane();
    test_brake();
    test_circle();
    test_drops();
    test_convergence();
    return finish("golden_unsprung");
}
