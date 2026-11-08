#include <cmath>
#include <cstdio>
#include <string>
#include "scenarios/bicycle_circle.hpp"
#include "scenarios/bicycle_lane_change.hpp"
#include "scenarios/bicycle_slalom.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// Golden values must match to this tolerance. It is far below any real change in the model,
// and loose enough that a different compiler's last-bit rounding does not fail the test.
static const Real golden_tol = 1e-9;

// Bit-for-bit equality of two states, for determinism tests.
static bool identical(const bicycle_state &a, const bicycle_state &b)
{
    return a.x == b.x && a.y == b.y && a.yaw == b.yaw && a.v == b.v;
}

// Compares a whole state with an expected one.
static void check_state(const std::string &what, const bicycle_state &a, const bicycle_state &e, Real t)
{
    check_near((what + " x").c_str(), a.x, e.x, t);
    check_near((what + " y").c_str(), a.y, e.y, t);
    check_near((what + " yaw").c_str(), a.yaw, e.yaw, t);
    check_near((what + " v").c_str(), a.v, e.v, t);
}

// Circle: 6 m/s, 0.2 rad of steer, one lap rounded to a whole number of 1 ms steps.
static void test_circle()
{
    const bicycle_run scenario = bicycle_circle();
    const bicycle_state s = run(scenario);

    check_true("circle: determinism: two runs give identical state", identical(s, run(bicycle_circle())));

    // Golden values, recorded at the phase 2 gate after checking them against the closed form below.
    const bicycle_state golden{0.00023200083662525822, 2.2576072430488319e-05, 6.2832041187559344, 6.0};
    check_state("circle golden", s, golden, golden_tol);

    // Closed form: the CG moves on a circle of radius R_cg = v / yaw_rate, whose centre is to the
    // left of the start, and after time t it is at R_cg * (sin(beta + w t) - sin(beta), cos(beta) - cos(beta + w t)).
    const bicycle_params &p = scenario.params;
    const Real v = 6.0, steer = 0.2, L = p.wheelbase();
    const Real beta = std::atan(p.lr / L * std::tan(steer));
    const Real w = v * std::cos(beta) * std::tan(steer) / L;
    const Real t = scenario.steps * scenario.dt;
    const Real r_cg = v / w;
    check_near("circle analytic: yaw = w t", s.yaw, w * t, 1e-9);
    check_near("circle analytic: x", s.x, r_cg * (std::sin(beta + w * t) - std::sin(beta)), 1e-6);
    check_near("circle analytic: y", s.y, r_cg * (std::cos(beta) - std::cos(beta + w * t)), 1e-6);
    check_near("circle analytic: speed unchanged", s.v, v, 1e-12);

    // The gate: one lap puts the car back at the start, within 1 cm.
    check_true("circle: back within 1 cm of the start", std::hypot(s.x, s.y) < 0.01);
    check_near("circle: heading turned by one lap", s.yaw, 2.0 * kPi, 1e-3);
}

// Lane change: 15 m/s, triangle-wave steer of 0.02 rad, 7 s.
static void test_lane_change()
{
    const bicycle_state s = run(bicycle_lane_change());

    check_true("lane: determinism: two runs give identical state", identical(s, run(bicycle_lane_change())));

    const bicycle_state golden{104.83361437256131, 3.5944882883404019, 8.6785142891103494e-18, 15.0};
    check_state("lane golden", s, golden, golden_tol);

    // Closed form for small angles (the steer is at most 0.02 rad, so tan(steer) = steer, cos(beta) = 1,
    // sin(psi) = psi to better than 0.3%). With k = v / L, the heading is psi = k * (integral of steer).
    // The steer is a triangle wave of height d, so:
    //   psi(3 s) = k d                      (the peak, after the left half)
    //   psi(7 s) = 0                        (the right half undoes the left half)
    //   y = v * integral of psi = 2 v^2 d / L
    //   x = v * (7 - 0.5 * integral of psi^2) = v * (7 - 0.5 * (23 / 15) * (k d)^2)
    const Real v = 15.0, L = 2.5, d = 0.02, k = v / L;
    check_near("lane analytic: heading is zero again", s.yaw, 0.0, 1e-9);
    check_near("lane analytic: y = 2 v^2 d / L", s.y, 2.0 * v * v * d / L, 0.01);
    check_near("lane analytic: x", s.x, v * (7.0 - 0.5 * (23.0 / 15.0) * (k * d) * (k * d)), 0.005);
    check_near("lane analytic: speed unchanged", s.v, v, 1e-12);

    bicycle_run half = bicycle_lane_change();
    half.steps = 3000;
    const bicycle_state m = run(half);
    check_near("lane analytic: peak heading at 3 s = k d", m.yaw, k * d, 1e-4);
}

// Slalom: 15 m/s, steer held at +-0.03 rad with fast ramps, 12 s.
static void test_slalom()
{
    const bicycle_state s = run(bicycle_slalom());

    check_true("slalom: determinism: two runs give identical state", identical(s, run(bicycle_slalom())));

    const bicycle_state golden{177.82363453606442, 0.016007269574794918, -1.5054147164961229e-16, 15.0};
    check_state("slalom golden", s, golden, golden_tol);

    // Closed form for small angles, as in the lane change. The holds are 1.35, 2.7, 2.7, 2.7 and
    // 1.35 s long with ramps of 0.3 s between them, so the heading swings evenly to both sides:
    //   psi(12 s) = 0,  y = 0 (the path ends on the centre line),
    //   psi(1.5 s) = 1.425 k d  (the first peak: 1.35 s of full steer plus half a ramp)
    //   x = 177.834 m (v * (12 - 0.5 * integral of psi^2), evaluated numerically outside the model)
    const Real v = 15.0, L = 2.5, d = 0.03, k = v / L;
    check_near("slalom analytic: heading is zero again", s.yaw, 0.0, 1e-9);
    check_near("slalom analytic: back on the centre line", s.y, 0.0, 0.03);
    check_near("slalom analytic: x", s.x, 177.834, 0.02);
    check_near("slalom analytic: speed unchanged", s.v, v, 1e-12);

    bicycle_run first = bicycle_slalom();
    first.steps = 1500;
    check_near("slalom analytic: first heading peak at 1.5 s", run(first).yaw, 1.425 * k * d, 5e-4);
}

int main()
{
    test_circle();
    test_lane_change();
    test_slalom();
    return finish("golden_bicycle");
}
