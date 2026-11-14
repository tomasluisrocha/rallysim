#include <cmath>
#include <cstdio>
#include <string>
#include "runner/builtin/single_track_circle.hpp"
#include "runner/builtin/single_track_lane_change.hpp"
#include "tests/check.hpp"
#include "tests/single_track_helpers.hpp"

using namespace rallysim;

// Golden values must match to this tolerance. It is far below any real change in the model,
// and loose enough that a different compiler's last-bit rounding does not fail the test.
static const Real golden_tol = 1e-9;

// The planar part of a single-track state: what the golden values record.
struct planar
{
    Real x, y, yaw, u, v, r;
};

static planar planar_of(const rigid_body_state &s)
{
    const body_velocity vel = velocity_in_body(s);
    return {s.pos.x, s.pos.y, yaw_angle(s.q), vel.u, vel.v, vel.r};
}

static void check_planar(const std::string &what, const planar &a, const planar &e, Real t)
{
    check_near((what + " x").c_str(), a.x, e.x, t);
    check_near((what + " y").c_str(), a.y, e.y, t);
    check_near((what + " yaw").c_str(), a.yaw, e.yaw, t);
    check_near((what + " u").c_str(), a.u, e.u, t);
    check_near((what + " v").c_str(), a.v, e.v, t);
    check_near((what + " r").c_str(), a.r, e.r, t);
}

// Prints a state in the form used for the golden values below, for re-recording after a deliberate change.
static void print_planar(const char *name, const planar &a)
{
    std::printf("%s: {%.17g, %.17g, %.17g, %.17g, %.17g, %.17g}\n", name, a.x, a.y, a.yaw, a.u, a.v, a.r);
}

// Circle: 20 s at 6 m/s (held), steer step to 0.2 rad at 1 s.
static void test_circle()
{
    const single_track_run scenario = single_track_circle();
    const rigid_body_state s = run(scenario);
    const planar got = planar_of(s);

    check_true("circle: determinism: two runs give identical state", identical(s, run(single_track_circle())));
    check_near("circle: the car stays on the ground plane", s.pos.z, 0.0, 1e-9);

    // Golden values, recorded after checking them against the closed form below.
    const planar golden{9.7586736908645921, 24.46252723414435, 2.7423215384616051,
                        5.9922881162235777, 0.42009771085295244, 0.47746727005745487};
    const int before_circle = failures;
    check_planar("circle golden", got, golden, golden_tol);
    if (failures != before_circle)
        print_planar("st_circle (re-record only after checking the closed form)", got);

    // Closed form: in the settled turn the steer is L / R + K a_y (small-angle form, so within a few
    // percent at 0.2 rad), and the speed controller holds the forward speed.
    const single_track_params &p = scenario.params;
    const Real v_cg = std::hypot(got.u, got.v);
    const Real radius = v_cg / got.r, a_y = v_cg * got.r;
    const Real predicted = p.wheelbase() / radius + understeer_gradient(p) * a_y;
    check_true("circle analytic: steer = L/R + K a_y within 5%", std::fabs(predicted / 0.2 - 1.0) < 0.05);
    check_near("circle analytic: speed held", got.u, 6.0, 0.05);
}

// Lane change: 7 s at 15 m/s (held), triangle-wave steer of 0.02 rad.
static void test_lane_change()
{
    const single_track_run scenario = single_track_lane_change();
    const rigid_body_state s = run(scenario);
    const planar got = planar_of(s);

    check_true("lane: determinism: two runs give identical state", identical(s, run(single_track_lane_change())));

    // Golden values, recorded after checking them against the closed form below.
    const planar golden{104.85479678495533, 3.3857424771665237, -4.9245689912291985e-08,
                        14.99999999999959, -6.6316534746647845e-09, 5.2403996387340423e-10};
    const int before_lane = failures;
    check_planar("lane golden", got, golden, golden_tol);
    if (failures != before_lane)
        print_planar("st_lane (re-record only after checking the closed form)", got);

    // The steer is odd about t = 3 s and back at zero for the last 2 s, so the car ends going straight
    // again, displaced to the left, after about 100 m.
    check_near("lane: yaw rate back to zero", got.r, 0.0, 1e-4);
    check_near("lane: sideways speed back to zero", got.v, 0.0, 1e-4);
    check_true("lane: displaced to the left", got.y > 0.5);
    check_near("lane: covered about 15 m/s * 7 s", got.x, 105.0, 1.0);
}

int main()
{
    test_circle();
    test_lane_change();
    return finish("golden_single_track");
}
