#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include "scenarios/four_wheel_circle.hpp"
#include "scenarios/four_wheel_driving.hpp"
#include "scenarios/four_wheel_lane_change.hpp"
#include "scenarios/single_track_circle.hpp"
#include "scenarios/single_track_lane_change.hpp"
#include "tests/check.hpp"
#include "tests/single_track_helpers.hpp"

using namespace rallysim;

// Golden runs for the four-wheel scenarios (phase 4, step 7), on springs. Each scenario is first checked against
// something that does not share its code (a closed form, or the single-track model), so a wrong result cannot be
// frozen in as golden; then its final state is compared with the values recorded at the phase 4 gate. Last, the
// timestep: halving dt must change the results less each time (book section 11.2).

// Golden values must match to this tolerance. It is far below any real change in the model,
// and loose enough that a different compiler's last-bit rounding does not fail the test.
static const Real golden_tol = 1e-9;

// What the golden values record: the body's position, heading, velocities, height, roll and pitch, and each
// wheel's spin.
struct snapshot
{
    Real x, y, yaw, u, v, r, z, roll, pitch, w_fl, w_fr, w_rl, w_rr;
};

static snapshot snapshot_of(const four_wheel_state &s)
{
    const body_velocity v = velocity_in_body(s.body);
    return {s.body.pos.x, s.body.pos.y, yaw_angle(s.body.q), v.u, v.v, v.r, s.body.pos.z,
            roll_angle(s.body.q), pitch_angle(s.body.q),
            s.wheels[FL].omega, s.wheels[FR].omega, s.wheels[RL].omega, s.wheels[RR].omega};
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

// Compares with the golden values; on a mismatch prints the new values in the form used below, for
// re-recording after a deliberate change (check the independent checks first).
static void check_golden(const char *name, const four_wheel_state &s, const snapshot &e)
{
    const snapshot a = snapshot_of(s);
    const Real got[] = {a.x, a.y, a.yaw, a.u, a.v, a.r, a.z, a.roll, a.pitch, a.w_fl, a.w_fr, a.w_rl, a.w_rr};
    const Real want[] = {e.x, e.y, e.yaw, e.u, e.v, e.r, e.z, e.roll, e.pitch, e.w_fl, e.w_fr, e.w_rl, e.w_rr};
    const char *fields[] = {"x", "y", "yaw", "u", "v", "r", "z", "roll", "pitch", "omega FL", "omega FR", "omega RL", "omega RR"};
    const int before = failures;
    for (int k = 0; k < 13; ++k)
        check_near((std::string(name) + " golden " + fields[k]).c_str(), got[k], want[k], golden_tol);
    if (failures != before)
    {
        std::printf("%s (re-record only after checking the independent checks):\n    {", name);
        for (int k = 0; k < 13; ++k)
            std::printf("%.17g%s", got[k], k < 12 ? ", " : "}\n");
    }
}

// Runs a scenario step by step, calling `watch` with the time and the state after each step.
static four_wheel_state trace(const four_wheel_run &r, const std::function<void(Real, const four_wheel_state &)> &watch)
{
    four_wheel_state s = r.initial;
    for (long i = 0; i < r.steps; ++i)
    {
        step(s, r.params, driver_input(r, s, i * r.dt), r.dt);
        watch((i + 1) * r.dt, s);
    }
    return s;
}

static const Real pi = 3.14159265358979323846;

// Circle: 20 s at 6 m/s (held), steer step to 0.2 rad at 1 s.
static void test_circle()
{
    const four_wheel_run r = four_wheel_circle();
    const four_wheel_state s = run(r);
    check_true("circle: determinism: two runs give identical bits", identical(s, run(four_wheel_circle())));
    check_golden("circle", s, {10.881383969291427, 24.257168527150991, 2.6529136336719104, 5.991757503621951,
        0.4197438892115759, 0.4728252244615358, 0.49995577845730993, 0.015957019841847081,
        0.00038943274580224667, 18.179633028095779, 20.386309223952463, 17.893534878756626, 20.146321643184308});

    // the settled turn: the steer is L / R + K a_y (small-angle form, so within a few percent at 0.2 rad); the body
    // leans out by m a_y h / (K_roll - m g h); the yaw rate is the single-track's within 2%
    const four_wheel_params &p = r.params;
    const body_velocity v = velocity_in_body(s.body);
    const Real speed = std::hypot(v.u, v.v), radius = speed / v.r, a_y = speed * v.r;
    single_track_params q;
    q.front_cornering_stiffness = 2 * p.front_tyre.cornering_stiffness;
    q.rear_cornering_stiffness = 2 * p.rear_tyre.cornering_stiffness;
    const Real predicted = p.wheelbase() / radius + understeer_gradient(q) * a_y;
    check_true("circle analytic: steer = L/R + K a_y within 5%", std::fabs(predicted / 0.2 - 1.0) < 0.05);
    const Real k_roll = (p.front_corner.spring_rate * p.track_front * p.track_front +
                         p.rear_corner.spring_rate * p.track_rear * p.track_rear) / 2;
    const Real roll = p.chassis.mass * a_y * p.cg_height / (k_roll - p.chassis.mass * kGravity * p.cg_height);
    check_near("circle analytic: roll = m a_y h / (K_roll - m g h) within 3%", roll_angle(s.body.q), roll, 0.03 * roll);
    const rigid_body_state single = run(single_track_circle());
    check_near("circle: yaw rate within 2% of the single-track's", v.r, single.omega.z, 0.02 * single.omega.z);
}

// Lane change: 7 s at 15 m/s (held), triangle-wave steer of 0.02 rad.
static void test_lane_change()
{
    const four_wheel_run r = four_wheel_lane_change();
    const four_wheel_state s = run(r);
    check_true("lane: determinism: two runs give identical bits", identical(s, run(four_wheel_lane_change())));
    check_golden("lane", s, {104.85133943103796, 3.3854321012588873, 2.7666543837507465e-08, 14.999999999592891,
        2.4194050936317824e-08, -6.2627363643480992e-10, 0.50000000000147504, -2.6044103170408835e-10,
        -6.6151143238857337e-12, 47.619047619200316, 47.61904761606165, 47.619047620035609, 47.619047616896907});

    // back to going straight, level and at speed, displaced to the left as the single-track is
    const body_velocity v = velocity_in_body(s.body);
    check_near("lane: yaw rate back to zero", v.r, 0.0, 1e-3);
    check_near("lane: sideways speed back to zero", v.v, 0.0, 1e-3);
    check_near("lane: body level again", roll_angle(s.body.q), 0.0, 1e-3);
    const rigid_body_state single = run(single_track_lane_change());
    check_near("lane: ends within 5 cm of where the single-track ends, sideways", s.body.pos.y, single.pos.y, 0.05);
    check_near("lane: and along the road", s.body.pos.x, single.pos.x, 0.05);
}

// Braking: 25 m/s, rolling for a second, then the brake fully down.
static void test_brake()
{
    const four_wheel_run r = four_wheel_brake();
    Real x_brake = 0, x_stop = -1, t_stop = -1;
    bool all_locked = false;
    const four_wheel_state s = trace(r, [&](Real t, const four_wheel_state &st)
                                     {
        if (t <= 1.0)
            x_brake = st.body.pos.x;
        bool locked = true;
        for (const wheel_state &w : st.wheels)
            locked = locked && w.omega == 0;
        all_locked = all_locked || locked;
        if (t_stop < 0 && velocity_in_body(st.body).u <= 0)
        {
            t_stop = t;
            x_stop = st.body.pos.x;
        } });
    check_true("brake: determinism: two runs give identical bits", identical(s, run(four_wheel_brake())));
    check_golden("brake", s, {57.773153252727504, 0, 0, 0.015712169314441179,
        0, 0, 0.49965502760450536, 0,
        -0.0019561948445738202, 0, 0, 0, 0});

    // the stop: about v^2 / (2 mu g) from where the brake went down (a few percent longer: the pedal takes 50 ms,
    // the tyres build their force over a relaxation length, and the loaded front loses a little grip to load
    // sensitivity); all four wheels lock; at the end the car is (nearly) still
    const Real expected = 25.0 * 25.0 / (2 * kGravity);
    std::printf("brake: stopped %.3f m after the brake went down, at %.3f s (v^2 / 2 mu g = %.3f m)\n", x_stop - x_brake,
                t_stop, expected);
    check_near("brake analytic: stopping distance within 4% of v^2 / (2 mu g)", x_stop - x_brake, expected, 0.04 * expected);
    check_true("brake: all four wheels lock", all_locked);
    check_near("brake: at rest at the end", velocity_in_body(s.body).u, 0.0, 0.05);
}

// Launch: four-wheel drive (30/70), from 2 m/s, full throttle from 0.5 s.
static void test_launch()
{
    const four_wheel_run r = four_wheel_launch();
    Real u_a = 0, u_b = 0, u_end_a = 0;
    const four_wheel_state s = trace(r, [&](Real t, const four_wheel_state &st)
                                     {
        const Real u = velocity_in_body(st.body).u;
        if (std::fabs(t - 0.8) < 1e-9) u_a = u;
        if (std::fabs(t - 1.2) < 1e-9) u_b = u;
        if (std::fabs(t - 3.4) < 1e-9) u_end_a = u; });
    check_true("launch: determinism: two runs give identical bits", identical(s, run(four_wheel_launch())));
    check_golden("launch", s, {47.58495227103537, 0, 0, 26.996153279568169,
        0, 0, 0.49939125516726851, 0,
        -0.011522889159315789, 86.669378751649504, 86.669378751649504, 88.101379088753916, 88.101379088753916});

    // traction-limited at first: about mu g (load sensitivity takes a little off); power-limited at the end:
    // a = P / (m_eff v), every wheel's spin inertia adding to the mass
    const Real early = (u_b - u_a) / 0.4;
    const Real u_end = velocity_in_body(s.body).u;
    const Real late = (u_end - u_end_a) / 0.1;
    const four_wheel_params &p = r.params;
    const Real m_eff = p.chassis.mass + 4 * p.wheel.inertia / (p.wheel.radius * p.wheel.radius);
    const Real power_limited = p.drive_brake.max_drive_power / (m_eff * 0.5 * (u_end + u_end_a));
    std::printf("launch: %.3f m/s^2 at first (mu g = %.3f), %.3f m/s^2 at the end (P / m_eff v = %.3f)\n", early,
                kGravity, late, power_limited);
    check_near("launch analytic: traction-limited at about mu g, within 8%", early, kGravity, 0.08 * kGravity);
    check_near("launch analytic: power-limited at P / (m_eff v) at the end, within 5%", late, power_limited, 0.05 * power_limited);
}

// Handbrake flick: 15 m/s held, a gentle left turn, a short partial pull of the handbrake, the wheel straightened.
static void test_handbrake()
{
    const four_wheel_run r = four_wheel_handbrake();
    Real peak_beta = 0;
    const four_wheel_state s = trace(r, [&](Real, const four_wheel_state &st)
                                     {
        const body_velocity v = velocity_in_body(st.body);
        peak_beta = std::max(peak_beta, std::fabs(std::atan2(v.v, v.u))); });
    check_true("handbrake: determinism: two runs give identical bits", identical(s, run(four_wheel_handbrake())));
    check_golden("handbrake", s, {70.962109842359226, 40.6981533657844, 0.99608716212553927, 14.999999965529085,
        -3.04243547688543e-07, -1.1415521074202461e-08, 0.50000000127889876, 3.0599667384784965e-08,
        -1.4964632752489423e-10, 47.619047533247368, 47.619047479142424, 47.619047603982196, 47.619047549877259});

    // the tail steps out, and the car recovers to going straight at the held speed
    const body_velocity v = velocity_in_body(s.body);
    std::printf("handbrake: peak sideslip %.1f deg\n", peak_beta * 180 / pi);
    check_true("handbrake: the tail steps out (sideslip between 5 and 15 degrees)", peak_beta > 5 * pi / 180 && peak_beta < 15 * pi / 180);
    check_near("handbrake: and the car recovers (sideslip back near zero)", std::atan2(v.v, v.u), 0.0, 2 * pi / 180);
    check_near("handbrake: at the held speed", v.u, 15.0, 0.1);
}

// Timestep convergence (book section 11.2): halving dt changes the result less each time. The same scenario at
// 2, 1 and 0.5 ms; semi-implicit Euler is first order, so each halving should roughly halve the change.
static void test_convergence()
{
    auto at = [](four_wheel_run r, Real dt)
    {
        const Real duration = r.steps * r.dt;
        r.dt = dt;
        r.steps = std::lround(duration / dt);
        return snapshot_of(run(r));
    };
    auto gap = [](const snapshot &a, const snapshot &b)
    { return std::hypot(a.x - b.x, a.y - b.y); };

    const snapshot lane2 = at(four_wheel_lane_change(), 0.002), lane1 = at(four_wheel_lane_change(), 0.001),
                   lane05 = at(four_wheel_lane_change(), 0.0005);
    const Real coarse = gap(lane2, lane1), fine = gap(lane1, lane05);
    std::printf("convergence, lane change: end position moves %.2e m from 2 to 1 ms, %.2e m from 1 to 0.5 ms\n", coarse, fine);
    check_true("convergence: lane change, halving dt changes the end position less each time", fine < 0.7 * coarse);
    check_true("convergence: lane change, by under 1 cm at 1 ms", coarse < 0.01);

    const snapshot brake2 = at(four_wheel_brake(), 0.002), brake1 = at(four_wheel_brake(), 0.001),
                   brake05 = at(four_wheel_brake(), 0.0005);
    const Real b_coarse = std::fabs(brake2.x - brake1.x), b_fine = std::fabs(brake1.x - brake05.x);
    std::printf("convergence, braking: stop moves %.2e m from 2 to 1 ms, %.2e m from 1 to 0.5 ms\n", b_coarse, b_fine);
    check_true("convergence: braking, halving dt changes where the car stops less each time", b_fine < 0.7 * b_coarse);
    check_true("convergence: braking, by under 5 cm at 1 ms", b_coarse < 0.05);
}

int main()
{
    test_circle();
    test_lane_change();
    test_brake();
    test_launch();
    test_handbrake();
    test_convergence();
    return finish("golden_four_wheel");
}
