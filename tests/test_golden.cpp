#include <cmath>
#include <cstdio>
#include <string>
#include "scenarios/driven_body.hpp"
#include "scenarios/thrown_body.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// Golden values must match to this tolerance. It is far below any real change in the physics,
// and loose enough that a different compiler's last-bit rounding does not fail the test.
static const Real golden_tol = 1e-9;

// Compares a whole state with an expected one, field group by field group.
static void check_state(const std::string &what, const rigid_body_state &a, const rigid_body_state &e, Real t)
{
    check_vec((what + " pos").c_str(), a.pos, e.pos, t);
    check_vec((what + " vel").c_str(), a.vel, e.vel, t);
    check_quat((what + " quat").c_str(), a.q, e.q, t);
    check_vec((what + " omega").c_str(), a.omega, e.omega, t);
}

// Thrown body: 3 s, dt = 1 ms. Force cancels gravity for 1 s, then free flight, with a constant
// yaw torque throughout.
static void test_thrown_body()
{
    const rigid_body_state a = run(thrown_body());

    // The same scenario must give identical bits.
    check_true("thrown: determinism: two runs give identical state", identical(a, run(thrown_body())));

    // Golden values, recorded from run(thrown_body()) at the phase 1 gate after checking them against
    // the closed form below. If a change moves them on purpose, re-check the new numbers by hand
    // and update them here.
    const rigid_body_state golden{{30.00000000000189, 0.0, -3.6298099999999605},
                                  {10.0, 0.0, -14.619999999999818},
                                  {0.69219613760730003, 0.0, 0.0, 0.72170943397016474},
                                  {0.0, 0.0, 0.57500000000015827}};
    check_state("thrown golden", a, golden, golden_tol);

    // Closed form, with the known first-order integration error as the allowed difference.
    // This stops a wrong result from being frozen in as "golden".
    const Real g = kGravity;
    check_near("thrown analytic pos.x = 10 m/s * 3 s", a.pos.x, 30.0, 1e-6);
    check_near("thrown analytic vel.z = 5 - g * 2 s", a.vel.z, 5.0 - g * 2.0, 1e-6);
    check_near("thrown analytic pos.z = 6 + 5*2 - g*2^2/2", a.pos.z, 6.0 + 5.0 * 2.0 - 0.5 * g * 4.0, 0.02);
    check_near("thrown analytic omega.z = 0.5 + (50/2000) * 3", a.omega.z, 0.5 + (50.0 / 2000.0) * 3.0, 1e-6);

    // Yaw angle = integral of omega.z. The simulator updates omega before q, so it overshoots by
    // about 0.5 * alpha * dt * T = 4e-5 rad (a 2e-5 error in the quaternion); 1e-4 allows for that.
    const Real yaw = 0.5 * 3.0 + 0.5 * (50.0 / 2000.0) * 9.0;
    check_near("thrown analytic quat.w = cos(yaw/2)", a.q.w, std::cos(0.5 * yaw), 1e-4);
    check_near("thrown analytic quat.z = sin(yaw/2)", a.q.z, std::sin(0.5 * yaw), 1e-4);
    check_near("thrown norm(q) = 1", norm(a.q), 1.0, 1e-12);
}

// Driven body: 9 s lane change. Sideways force +1500 N then -1500 N, no torque.
static void test_driven_body()
{
    const rigid_body_state d = run(driven_body());

    check_true("driven: determinism: two runs give identical state", identical(d, run(driven_body())));

    // Golden values, recorded at the phase 1 gate after checking them against the closed form below.
    const rigid_body_state golden{{45.000000000004569, 11.250000000000245, -4.8942183639151091e-17},
                                  {5.0, 8.4637459089604693e-16, -5.3290705182007515e-18},
                                  {1.0, 0.0, 0.0, 0.0},
                                  {0.0, 0.0, 0.0}};
    check_state("driven golden", d, golden, golden_tol);

    // Closed form: x moves at 5 m/s for 9 s. The sideways force is odd-symmetric about t = 4.5 s,
    // so vel.y returns to zero and the body ends 11.25 m to the left (the area under vel.y).
    check_near("driven analytic pos.x = 5 m/s * 9 s", d.pos.x, 45.0, 1e-6);
    check_near("driven analytic pos.y = 11.25 m", d.pos.y, 11.25, 1e-6);
    check_near("driven analytic vel.y returns to 0", d.vel.y, 0.0, 1e-9);
    check_near("driven analytic height stays 0 (lift cancels gravity)", d.pos.z, 0.0, 1e-9);
    check_quat("driven analytic heading never changes", d.q, quat{}, 1e-12);

    // Halfway (t = 4.5 s) the force changes sign: vel.y peaks at 3.4375 m/s and the body is
    // half way across (5.625 m).
    scripted_run half = driven_body();
    half.steps = 4500;
    const rigid_body_state m = run(half);
    check_near("driven analytic vel.y peak at 4.5 s", m.vel.y, 3.4375, 1e-6);
    check_near("driven analytic half the lateral distance at 4.5 s", m.pos.y, 5.625, 1e-6);
}

int main()
{
    test_thrown_body();
    test_driven_body();
    return finish("golden");
}
