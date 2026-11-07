#include <cmath>
#include <cstdio>
#include "core/rigid_body.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// A force that exactly cancels gravity, so a test can isolate rotation or a single push.
static wrench hover(const rigid_body_params &p, const vec3 &extra_force = vec3{}, const vec3 &torque = vec3{})
{
    return wrench{extra_force + vec3{0, 0, p.mass * kGravity}, torque};
}

// Runs a torque-free spin with an off-axis angular velocity and returns the worst relative drift
// of the world-frame angular momentum over `seconds`.
static Real angular_momentum_drift(Real dt, Real seconds)
{
    rigid_body_params p;
    p.inertia = {1.0, 2.0, 3.0};
    rigid_body_state s;
    s.omega = {3.0, 1.0, 0.5};
    const wrench w = hover(p);

    auto world_L = [&](const rigid_body_state &st)
    {
        const vec3 Iw{p.inertia.x * st.omega.x, p.inertia.y * st.omega.y, p.inertia.z * st.omega.z};
        return rotate(st.q, Iw);
    };
    const vec3 L0 = world_L(s);
    Real worst = 0;
    const int steps = static_cast<int>(seconds / dt + 0.5);
    for (int i = 0; i < steps; ++i)
    {
        step(s, p, w, dt);
        worst = std::fmax(worst, norm(world_L(s) - L0) / norm(L0));
    }
    check_near("off-axis spin keeps norm(q) = 1", norm(s.q), 1.0);
    return worst;
}

// A scripted scenario with time-varying force and torque; returns the final state.
static rigid_body_state scripted_run()
{
    rigid_body_params p;
    p.mass = 1200.0;
    p.inertia = {500.0, 1800.0, 2000.0};
    rigid_body_state s;
    s.vel = {10.0, 0.0, 5.0};
    s.omega = {0.1, 0.2, 0.5};
    for (int i = 0; i < 5000; ++i)
    {
        const wrench w{vec3{1000.0 * std::sin(0.001 * i), 200.0, p.mass * kGravity},
                       vec3{50.0 * std::cos(0.002 * i), 0.0, 100.0}};
        step(s, p, w, 0.001);
    }
    return s;
}

int main()
{
    // --- Test 1: free fall -------------------------------------------------------------------
    // Semi-implicit Euler has a known first-order error:
    //   z = z0 - 0.5 g t^2 - 0.5 g t dt   (about -4.9 mm at t = 1 s, dt = 1 ms)
    // so the test asserts that term, not zero.
    {
        const Real dt = 0.001, z0 = 100.0;
        const int steps = 1000;
        rigid_body_params p;
        rigid_body_state s;
        s.pos.z = z0;
        for (int i = 0; i < steps; ++i)
            step(s, p, wrench{}, dt);
        const Real t = steps * dt;
        const Real ideal = z0 - 0.5 * kGravity * t * t;
        const Real known_error = -0.5 * kGravity * t * dt;
        check_near("free fall: velocity = -g t", s.vel.z, -kGravity * t, 1e-9);
        check_near("free fall: position error equals 0.5 g t dt", s.pos.z - ideal, known_error,
                   0.01 * std::fabs(known_error));
        check_near("free fall: no sideways drift", s.pos.x + s.pos.y, 0.0);
    }

    // --- Test 2: zero force and constant force -----------------------------------------------
    {
        const Real dt = 0.001;
        rigid_body_params p;
        p.mass = 2.0;
        rigid_body_state s;
        for (int i = 0; i < 2000; ++i)
            step(s, p, hover(p), dt);
        check_vec("force cancelling gravity: velocity stays zero", s.vel, vec3{0, 0, 0}, 1e-9);

        for (int i = 0; i < 1000; ++i)
            step(s, p, hover(p, vec3{10.0, 0, 0}), dt); // 10 N / 2 kg = 5 m/s^2 along x
        check_near("constant force: v = F/m t", s.vel.x, 5.0, 1e-9);
        check_near("constant force: no y or z velocity", s.vel.y + s.vel.z, 0.0, 1e-9);
    }

    // --- Test 3: constant torque about a principal axis --------------------------------------
    {
        const Real dt = 0.001;
        rigid_body_params p;
        p.inertia = {1.0, 2.0, 3.0};
        rigid_body_state s;
        for (int i = 0; i < 1000; ++i)
            step(s, p, hover(p, vec3{}, vec3{0, 0, 6.0}), dt); // 6 / Iz(3) = 2 rad/s^2
        check_near("constant torque: omega.z = (tau/I) t", s.omega.z, 2.0, 1e-9);
        check_near("constant torque: omega.x stays 0", s.omega.x, 0.0);
        check_near("constant torque: omega.y stays 0", s.omega.y, 0.0);
        check_near("constant torque: norm(q) = 1", norm(s.q), 1.0);
    }

    // --- Test 4: torque-free spin about a principal axis -------------------------------------
    {
        const Real dt = 0.001;
        rigid_body_params p;
        p.inertia = {1.0, 2.0, 3.0};
        rigid_body_state s;
        s.omega = {0, 0, 5.0};
        for (int i = 0; i < 10000; ++i)
            step(s, p, hover(p), dt);
        check_vec("principal-axis spin: omega unchanged", s.omega, vec3{0, 0, 5.0});
        check_near("principal-axis spin: norm(q) = 1", norm(s.q), 1.0);
    }

    // --- Test 5: off-axis spin --------------------------------------------------------------
    // The body precesses, so omega is NOT constant here. What must hold is that the world-frame
    // angular momentum stays (nearly) constant. Semi-implicit Euler is only first order, so it
    // drifts: the 1% limit below was set after measuring the drift, and halving dt must shrink it.
    {
        const Real d1 = angular_momentum_drift(0.001, 10.0);
        const Real d2 = angular_momentum_drift(0.0005, 10.0);
        std::printf("angular-momentum drift: dt=1 ms %.3g, dt=0.5 ms %.3g\n", d1, d2);
        check_true("off-axis spin: drift under 1% at 1 ms", d1 < 1e-2);
        check_true("off-axis spin: halving dt shrinks the drift", d2 < 0.6 * d1);
    }

    // --- Test 6: determinism ----------------------------------------------------------------
    // Same inputs on the same machine must give identical bits, so exact comparison is correct.
    {
        const rigid_body_state a = scripted_run();
        const rigid_body_state b = scripted_run();
        check_true("determinism: two identical runs give identical state", identical(a, b));
    }

    return finish("rigid_body");
}