#include <cmath>
#include <cstdio>
#include "vehicle/bicycle.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// Runs a constant-steer, constant-speed turn for exactly half a lap in `n` steps and returns the
// final state. The step is T_half / n, so the half lap ends exactly on a step.
static bicycle_state half_lap(const bicycle_params &p, Real steer, Real v, int n)
{
    const Real L = p.wheelbase();
    const Real beta = std::atan(p.lr / L * std::tan(steer));
    const Real yaw_rate = v * std::cos(beta) * std::tan(steer) / L;
    const Real dt = (kPi / yaw_rate) / n;
    bicycle_state s;
    s.v = v;
    for (int i = 0; i < n; ++i)
        step(s, p, bicycle_input{steer, 0.0}, dt);
    return s;
}

// Position error of the half lap against the closed form: the CG moves on a circle through the
// start point, and half a lap later it is on the opposite side of the centre.
static Real half_lap_error(const bicycle_params &p, Real steer, Real v, int n)
{
    const bicycle_state s = half_lap(p, steer, v, n);
    const Real L = p.wheelbase();
    const Real beta = std::atan(p.lr / L * std::tan(steer));
    const Real r_cg = std::hypot(p.lr, L / std::tan(steer));
    const vec3 centre{-r_cg * std::sin(beta), r_cg * std::cos(beta), 0.0}; // left of the velocity
    return norm(vec3{s.x, s.y, 0.0} - 2.0 * centre);
}

int main()
{
    const bicycle_params p; // lf = 1.3, lr = 1.2
    const Real dt = 0.001;

    check_near("wheelbase = lf + lr", p.wheelbase(), 2.5);

    // straight line: steering zero keeps the heading, and the CG moves v t along it
    {
        bicycle_state s;
        s.yaw = 0.5;
        s.v = 10.0;
        for (int i = 0; i < 1000; ++i)
            step(s, p, bicycle_input{}, dt);
        check_near("straight: x = v t cos(yaw)", s.x, 10.0 * std::cos(0.5), 1e-9);
        check_near("straight: y = v t sin(yaw)", s.y, 10.0 * std::sin(0.5), 1e-9);
        check_near("straight: yaw unchanged", s.yaw, 0.5);
        check_near("straight: speed unchanged", s.v, 10.0);
    }

    // constant acceleration along the heading: v = a t, x = 0.5 a t^2 + 0.5 a t dt (speed is
    // updated before position, as in the rigid body)
    {
        bicycle_state s;
        for (int i = 0; i < 1000; ++i)
            step(s, p, bicycle_input{0.0, 2.0}, dt);
        check_near("accel: v = a t", s.v, 2.0, 1e-9);
        check_near("accel: x = 0.5 a t^2 + 0.5 a t dt", s.x, 0.5 * 2.0 * 1.0 + 0.5 * 2.0 * 1.0 * dt, 1e-9);
        check_near("accel: y stays zero", s.y, 0.0);
    }

    // zero speed: the car does not move, however the wheel is steered
    {
        bicycle_state s;
        for (int i = 0; i < 100; ++i)
            step(s, p, bicycle_input{0.4, 0.0}, dt);
        check_near("standstill: x", s.x, 0.0);
        check_near("standstill: y", s.y, 0.0);
        check_near("standstill: yaw", s.yaw, 0.0);
    }

    // mirror: steering left and right give mirrored paths
    {
        bicycle_state a, b;
        a.v = b.v = 10.0;
        for (int i = 0; i < 1000; ++i)
        {
            step(a, p, bicycle_input{0.2, 0.0}, dt);
            step(b, p, bicycle_input{-0.2, 0.0}, dt);
        }
        check_true("mirror: exact", a.x == b.x && a.y == -b.y && a.yaw == -b.yaw);
    }

    // left turn goes left
    {
        bicycle_state s;
        s.v = 10.0;
        for (int i = 0; i < 100; ++i)
            step(s, p, bicycle_input{0.2, 0.0}, dt);
        check_true("positive steer turns left (y > 0, yaw > 0)", s.y > 0.0 && s.yaw > 0.0);
    }

    // half lap at 0.2 rad, 10 m/s (4000 steps of about 1 ms): the position matches the circle to
    // about a micrometre, and the heading has turned by exactly pi
    {
        const Real err = half_lap_error(p, 0.2, 10.0, 4000);
        std::printf("half-lap position error with 4000 steps: %.3g m\n", err);
        check_true("half lap: position within 2e-6 m", err < 2e-6);
        check_near("half lap: heading turned by pi", half_lap(p, 0.2, 10.0, 4000).yaw, kPi, 1e-9);
    }

    // converges: doubling the number of steps must shrink the error by about 4 (second order)
    {
        const Real e1 = half_lap_error(p, 0.2, 10.0, 4000);
        const Real e2 = half_lap_error(p, 0.2, 10.0, 8000);
        check_true("half lap: halving dt shrinks the error by about 4", e2 < 0.3 * e1);
    }

    // radius of each point: the CG runs on its own circle, the rear axle on L / tan(steer), the
    // front wheel on L / sin(steer)
    {
        const Real steer = 0.2, L = p.wheelbase();
        const bicycle_state s = half_lap(p, steer, 10.0, 4000);
        const Real beta = std::atan(p.lr / L * std::tan(steer));
        const Real r_cg = std::hypot(p.lr, L / std::tan(steer));
        const vec3 centre{-r_cg * std::sin(beta), r_cg * std::cos(beta), 0.0};
        const vec3 cg{s.x, s.y, 0.0};
        const vec3 heading{std::cos(s.yaw), std::sin(s.yaw), 0.0};
        const vec3 rear = cg - p.lr * heading;
        const vec3 front = cg + p.lf * heading;
        check_near("radius of the CG", norm(cg - centre), r_cg, 2e-6);
        check_near("radius of the rear axle = L / tan(steer)", norm(rear - centre), L / std::tan(steer), 2e-6);
        check_near("radius of the front wheel = L / sin(steer)", norm(front - centre), L / std::sin(steer), 2e-6);
    }

    // lr = 0: the CG is the rear axle, so it runs on L / tan(steer) and the heading is the velocity
    {
        bicycle_params rear_axle;
        rear_axle.lf = 2.5;
        rear_axle.lr = 0.0;
        const Real err = half_lap_error(rear_axle, 0.2, 10.0, 4000);
        check_true("lr = 0: half lap position within 2e-6 m", err < 2e-6);
        const Real r = 2.5 / std::tan(0.2);
        const bicycle_state s = half_lap(rear_axle, 0.2, 10.0, 4000);
        check_near("lr = 0: after half a lap the CG is 2R to the left", s.y, 2.0 * r, 2e-6);
        check_near("lr = 0: and back at x = 0", s.x, 0.0, 2e-6);
    }

    // full lap: the heading comes back to 2 pi and the car to its start. For the rear-axle form
    // (lr = 0) the lap time is 2 pi R / v with R = L / tan(steer). For the CG form
    // it is 2 pi R_cg / v.
    {
        bicycle_params rear_axle;
        rear_axle.lf = 2.5;
        rear_axle.lr = 0.0;
        const Real steer = 0.2, v = 10.0;
        const Real R = 2.5 / std::tan(steer);
        const Real T = 2.0 * kPi * R / v;
        const int n = static_cast<int>(std::lround(T / dt)); // dt = 1 ms, so the lap may end up to half a step off
        bicycle_state s;
        s.v = v;
        for (int i = 0; i < n; ++i)
            step(s, rear_axle, bicycle_input{steer, 0.0}, dt);
        check_true("lap (rear axle): back within 1 cm of the start at dt = 1 ms", std::hypot(s.x, s.y) < 0.01);

        const Real L = p.wheelbase();
        const Real beta = std::atan(p.lr / L * std::tan(steer));
        const Real T_cg = 2.0 * kPi * std::hypot(p.lr, L / std::tan(steer)) / v;
        check_near("lap (CG): period = 2 pi R_cg / v equals 2 pi / yaw rate",
                   T_cg, 2.0 * kPi / (v * std::cos(beta) * std::tan(steer) / L), 1e-12);
        const int m = static_cast<int>(std::lround(T_cg / dt));
        bicycle_state c;
        c.v = v;
        for (int i = 0; i < m; ++i)
            step(c, p, bicycle_input{steer, 0.0}, dt);
        check_true("lap (CG): back within 1 cm of the start at dt = 1 ms", std::hypot(c.x, c.y) < 0.01);
        check_near("lap (CG): heading turned by 2 pi", c.yaw, 2.0 * kPi, 1e-2);
    }

    // reversing: with negative speed the heading turns the other way, and driving backwards for as
    // long as the car drove forwards (same steer) retraces the path back to the start
    {
        bicycle_state s;
        s.v = -5.0;
        for (int i = 0; i < 1000; ++i)
            step(s, p, bicycle_input{0.3, 0.0}, dt);
        check_true("reverse: left steer turns the heading right (yaw < 0)", s.yaw < 0.0);
        check_true("reverse: moves backwards (x < 0)", s.x < 0.0);

        bicycle_state r;
        r.v = 5.0;
        for (int i = 0; i < 1000; ++i)
            step(r, p, bicycle_input{0.3, 0.0}, dt);
        r.v = -5.0;
        for (int i = 0; i < 1000; ++i)
            step(r, p, bicycle_input{0.3, 0.0}, dt);
        check_vec("reverse: forward then backward retraces to the start",
                  vec3{r.x, r.y, r.yaw}, vec3{0.0, 0.0, 0.0}, 1e-9);
    }

    // steer saturation: an input beyond max_steer acts exactly like max_steer
    {
        bicycle_params q = p;
        q.max_steer = 0.5;
        bicycle_state big, edge, neg_big, neg_edge, inside, inside_ref;
        big.v = edge.v = neg_big.v = neg_edge.v = inside.v = inside_ref.v = 10.0;
        for (int i = 0; i < 500; ++i)
        {
            step(big, q, bicycle_input{3.0, 0.0}, dt);
            step(edge, q, bicycle_input{0.5, 0.0}, dt);
            step(neg_big, q, bicycle_input{-3.0, 0.0}, dt);
            step(neg_edge, q, bicycle_input{-0.5, 0.0}, dt);
            step(inside, q, bicycle_input{0.3, 0.0}, dt);
            step(inside_ref, p, bicycle_input{0.3, 0.0}, dt); // p has max_steer 0.6, so 0.3 is not clamped
        }
        check_true("saturation: +3 rad acts as +max_steer", big.x == edge.x && big.y == edge.y && big.yaw == edge.yaw);
        check_true("saturation: -3 rad acts as -max_steer", neg_big.x == neg_edge.x && neg_big.y == neg_edge.y &&
                                                                neg_big.yaw == neg_edge.yaw);
        check_true("saturation: a steer below the limit is not changed", inside.x == inside_ref.x &&
                                                                             inside.yaw == inside_ref.yaw);
        check_true("saturation: the position stays finite for a huge input", std::isfinite(big.x) && std::isfinite(big.yaw));
    }

    return finish("bicycle");
}
