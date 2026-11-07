#include <cmath>
#include <cstdio>
#include "scenarios/thrown_body.hpp"
#include "tests/check.hpp"

using namespace rallysim;


int main()
{
    const rigid_body_state a = run(thrown_body());

    // --- Determinism: the same scenario must give identical bits ----------------------------
    check_true("determinism: two runs give identical state", identical(a, run(thrown_body())));

    // --- Golden values -----------------------------------------------------------------------
    // Recorded from run_thrown_body() (3 s, dt = 1 ms) at the phase 1 gate, after checking them
    // against the closed-form answers in the next block. If a change moves these on purpose,
    // re-check the new numbers by hand and update them here.
    const Real tol = 1e-9;
    check_near("golden pos.x", a.pos.x, 30.00000000000189, tol);
    check_near("golden pos.y", a.pos.y, 0.0, tol);
    check_near("golden pos.z", a.pos.z, -3.6298099999999605, tol);
    check_near("golden vel.x", a.vel.x, 10.0, tol);
    check_near("golden vel.y", a.vel.y, 0.0, tol);
    check_near("golden vel.z", a.vel.z, -14.619999999999818, tol);
    check_near("golden quat.w", a.q.w, 0.69219613760730003, tol);
    check_near("golden quat.x", a.q.x, 0.0, tol);
    check_near("golden quat.y", a.q.y, 0.0, tol);
    check_near("golden quat.z", a.q.z, 0.72170943397016474, tol);
    check_near("golden omega.x", a.omega.x, 0.0, tol);
    check_near("golden omega.y", a.omega.y, 0.0, tol);
    check_near("golden omega.z", a.omega.z, 0.57500000000015827, tol);

    // --- Closed-form cross-check ---------------------------------------------------------------
    // Where the golden numbers come from, with the known first-order integration error as the
    // allowed difference. This stops a wrong result from being frozen in as "golden".
    const Real g = kGravity;
    check_near("analytic pos.x = 10 m/s * 3 s", a.pos.x, 30.0, 1e-6);
    check_near("analytic vel.z = 5 - g * 2 s", a.vel.z, 5.0 - g * 2.0, 1e-6);
    check_near("analytic pos.z = 6 + 5*2 - g*2^2/2", a.pos.z, 6.0 + 5.0 * 2.0 - 0.5 * g * 4.0, 0.02);
    check_near("analytic omega.z = 0.5 + (50/2000) * 3", a.omega.z, 0.5 + (50.0 / 2000.0) * 3.0, 1e-6);
    // yaw angle = integral of omega.z. The simulator updates omega before q, so it overshoots by
    // about 0.5 * alpha * dt * T = 4e-5 rad (a 2e-5 error in the quaternion); 1e-4 allows for that.
    const Real yaw = 0.5 * 3.0 + 0.5 * (50.0 / 2000.0) * 9.0;
    check_near("analytic quat.w = cos(yaw/2)", a.q.w, std::cos(0.5 * yaw), 1e-4);
    check_near("analytic quat.z = sin(yaw/2)", a.q.z, std::sin(0.5 * yaw), 1e-4);
    check_near("norm(q) = 1", norm(a.q), 1.0, 1e-12);

    return finish("golden");
}