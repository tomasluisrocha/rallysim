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

        // Zero net force: a body that is already moving keeps its velocity and goes in a straight line.
        rigid_body_state s;
        s.pos = {1.0, -1.0, 2.0};
        s.vel = {3.0, -2.0, 0.5};
        for (int i = 0; i < 2000; ++i)
            step(s, p, hover(p), dt);
        check_vec("zero force: velocity stays constant", s.vel, vec3{3.0, -2.0, 0.5}, 1e-9);
        check_vec("zero force: position = x0 + v t", s.pos, vec3{1.0 + 6.0, -1.0 - 4.0, 2.0 + 1.0}, 1e-9);

        // Constant force: a = F/m. Semi-implicit Euler gives x = 0.5 a t^2 + 0.5 a t dt exactly,
        // the same first-order term as in free fall.
        rigid_body_state r;
        for (int i = 0; i < 1000; ++i)
            step(r, p, hover(p, vec3{10.0, 4.0, 0}), dt); // 5 m/s^2 along x, 2 m/s^2 along y
        const Real t = 1000 * dt;
        check_near("constant force: v.x = F/m t", r.vel.x, 5.0 * t, 1e-9);
        check_near("constant force: v.y = F/m t", r.vel.y, 2.0 * t, 1e-9);
        check_near("constant force: v.z stays zero", r.vel.z, 0.0, 1e-9);
        check_near("constant force: x = 0.5 a t^2 + 0.5 a t dt", r.pos.x, 0.5 * 5.0 * t * t + 0.5 * 5.0 * t * dt, 1e-9);
        check_near("constant force: y = 0.5 a t^2 + 0.5 a t dt", r.pos.y, 0.5 * 2.0 * t * t + 0.5 * 2.0 * t * dt, 1e-9);
        check_near("constant force: z stays zero", r.pos.z, 0.0, 1e-9);
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

        // The angle turned is 0.5 alpha t^2 + 0.5 alpha t dt (omega is updated before q).
        // Measured error of the orientation is 2e-7; 1e-6 is the limit used here.
        const Real alpha = 2.0, t = 1000 * dt;
        const Real angle = 0.5 * alpha * t * t + 0.5 * alpha * t * dt;
        check_vec("constant torque: orientation = angle about z", rotate(s.q, vec3{1, 0, 0}),
                  vec3{std::cos(angle), std::sin(angle), 0}, 1e-6);
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

        // Orientation after 1 s at 5 rad/s (5 rad turned). Measured error 1e-5; 5e-5 is the limit.
        rigid_body_state r;
        r.omega = {0, 0, 5.0};
        for (int i = 0; i < 1000; ++i)
            step(r, p, hover(p), dt);
        check_vec("principal-axis spin: orientation = omega t", rotate(r.q, vec3{1, 0, 0}),
                  vec3{std::cos(5.0), std::sin(5.0), 0}, 5e-5);
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