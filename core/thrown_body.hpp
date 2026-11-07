#pragma once
#include "core/rigid_body.hpp"
#include "core/rigid_body_telemetry.hpp"
#include "core/telemetry.hpp"

namespace rallysim
{
    inline wrench wrench_at(long i, const rigid_body_params &p)
    {
        const bool hovering = i < 1000; // cancel gravity for the first 1000 frames
        return wrench{vec3{0, 0, hovering ? p.mass * kGravity : 0.0},
                      vec3{0, 0, 50.0}};
    }

    // runs the thrown-body scenario and returns the final state
    inline rigid_body_state run_thrown_body(telemetry_writer *writer = nullptr)
    {
        const Real dt = 0.001;
        const long steps = 3000;

        rigid_body_params p;
        p.mass = 1200.0;
        p.inertia = {500.0, 1800.0, 2000.0};

        rigid_body_state s;
        s.pos = {0, 0, 1};
        s.vel = {10, 0, 5};
        s.omega = {0, 0, 0.5};

        for (long i = 0; i < steps; ++i)
        {
            if (writer)
                writer->row(rigid_body_row(i, dt, s));
            step(s, p, wrench_at(i, p), dt);
        }
        if (writer)
            writer->row(rigid_body_row(steps, dt, s));
        return s;
    }
}