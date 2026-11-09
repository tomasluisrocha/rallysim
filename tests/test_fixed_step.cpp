#include <cmath>
#include "core/fixed_step.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// The real-time loop (core/fixed_step.hpp): whole steps from the accumulated frame time, the spiral-of-death
// clamp, and the render interpolation between the last two states.
int main()
{
    // 60 Hz frames over one simulated second give 1000 steps of 1 ms in total
    {
        fixed_step_clock clock{0.001};
        long steps = 0;
        for (int frame = 0; frame < 60; ++frame)
            steps += clock.advance(1.0 / 60.0);
        check_true("60 frames of 1/60 s run 1000 steps", steps == 1000);
        check_true("the accumulator stays below one step", clock.accumulator >= 0 && clock.accumulator < clock.dt);
    }

    // a frame shorter than a step runs no step and keeps the time for the next frame
    {
        fixed_step_clock clock{0.001};
        check_true("0.4 ms frame: no step", clock.advance(0.0004) == 0);
        check_near("0.4 ms frame: alpha = 0.4", clock.alpha(), 0.4, 1e-9);
        check_true("another 0.7 ms: one step", clock.advance(0.0007) == 1);
        check_near("and 0.1 ms left over", clock.accumulator, 0.0001, 1e-12);
    }

    // a stall (2 s frame) is clamped to max_frame: 50 steps, not 2000
    {
        fixed_step_clock clock{0.001};
        check_true("2 s frame runs only max_frame / dt = 50 steps", clock.advance(2.0) == 50);
        check_true("negative frame time runs nothing", clock.advance(-1.0) == 0);
    }

    // interpolation: the end points are the two states, the middle is half way
    {
        rigid_body_state a, b;
        a.pos = {0, 0, 0};
        b.pos = {2, 4, -6};
        b.vel = {1, 0, 0};
        b.q = from_axis_angle(vec3{0, 0, 1}, 0.2);
        check_vec("t = 0 gives a", interpolate(a, b, 0.0).pos, a.pos);
        check_vec("t = 1 gives b", interpolate(a, b, 1.0).pos, b.pos);
        check_quat("t = 1 gives b's orientation", interpolate(a, b, 1.0).q, b.q, 1e-12);
        check_vec("t = 0.5 is half way", interpolate(a, b, 0.5).pos, vec3{1, 2, -3});
        check_vec("velocity too", interpolate(a, b, 0.5).vel, vec3{0.5, 0, 0});

        // half way between 0 and 0.2 rad of yaw is 0.1 rad
        check_vec("orientation half way", to_world(interpolate(a, b, 0.5).q, vec3{1, 0, 0}),
                  vec3{std::cos(0.1), std::sin(0.1), 0}, 1e-12);

        // -q is the same orientation as q; the blend must not swing the long way round
        rigid_body_state c = b;
        c.q = quat{-b.q.w, -b.q.x, -b.q.y, -b.q.z};
        check_vec("orientation half way with the sign of q flipped", to_world(interpolate(a, c, 0.5).q, vec3{1, 0, 0}),
                  vec3{std::cos(0.1), std::sin(0.1), 0}, 1e-12);
    }

    return finish("fixed_step");
}
