#pragma once
#include <algorithm>
#include <cmath>
#include "core/rigid_body.hpp"

namespace rallysim
{
    // Real-time loop for a fixed physics step (after Fiedler's "Fix Your Timestep").
    // Real time is added to an accumulator each frame and spent in whole steps of dt; what is left is the
    // fraction of a step to interpolate the render pose by. The physics itself only ever sees dt.
    //
    //   fixed_step_clock clock{0.001};
    //   while (running) {
    //       for (int n = clock.advance(frame_time); n > 0; --n) {
    //           previous = state;
    //           step(state, params, latched_input(), clock.dt);
    //       }
    //       render(interpolate(previous, state, clock.alpha()));
    //   }
    struct fixed_step_clock
    {
        Real dt = 0.001;        // s, the physics step
        Real max_frame = 0.05;  // s, longest frame time taken into account (spiral-of-death guard)
        Real accumulator = 0;   // s, real time not yet simulated, always in [0, dt)

        // Adds one frame of real time and returns how many physics steps to run now. A frame longer than
        // max_frame counts as max_frame, so a stall slows the simulation down instead of making the next
        // frame run ever more steps. Negative frame times count as zero. Time within 1e-9 of a whole
        // number of steps counts as that many steps, so rounding in dt (0.001 is not exact in binary)
        // does not hold back a step.
        int advance(Real frame_time)
        {
            accumulator += std::clamp(frame_time, static_cast<Real>(0), max_frame);
            const int steps = static_cast<int>(std::floor(accumulator / dt + 1e-9));
            accumulator = std::max(accumulator - steps * dt, static_cast<Real>(0));
            return steps;
        }

        // How far the real time is between the last two physics states, 0 to 1.
        Real alpha() const { return accumulator / dt; }
    };

    // Render pose between two physics states, t = 0 gives a, t = 1 gives b. Positions and velocities are
    // interpolated linearly, the orientation along the shorter way (normalised linear interpolation, which
    // is close enough to slerp for the small turn of one step). For display only, never fed back.
    inline rigid_body_state interpolate(const rigid_body_state &a, const rigid_body_state &b, Real t)
    {
        rigid_body_state s;
        s.pos = a.pos + (b.pos - a.pos) * t;
        s.vel = a.vel + (b.vel - a.vel) * t;
        s.omega = a.omega + (b.omega - a.omega) * t;

        // q and -q are the same orientation: flip b if needed so the blend goes the short way
        const Real d = a.q.w * b.q.w + a.q.x * b.q.x + a.q.y * b.q.y + a.q.z * b.q.z;
        const Real sign = d < 0 ? -1 : 1;
        const Real u = 1 - t, v = sign * t;
        s.q = normalized(quat{u * a.q.w + v * b.q.w, u * a.q.x + v * b.q.x,
                              u * a.q.y + v * b.q.y, u * a.q.z + v * b.q.z});
        return s;
    }
}
