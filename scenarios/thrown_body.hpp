#pragma once
#include "core/rigid_body.hpp"
#include "core/rigid_body_telemetry.hpp"
#include "core/telemetry.hpp"
#include "core/wrench_script.hpp"

namespace rallysim
{
    inline wrench_script thrown_body_script(const rigid_body_params &p)
    {
        const wrench hover{vec3{0, 0, p.mass * kGravity}, vec3{0, 0, 50.0}};
        const wrench coast{vec3{0, 0, 0}, vec3{0, 0, 50.0}};
        wrench_script s;
        s.keys = {{0.0, hover}, {0.999, hover}, {1.0, coast}};
        return s;
    }

    // runs the thrown-body scenario and returns the final state
    inline rigid_body_state run_thrown_body(telemetry_writer *writer = nullptr)
    {
        const Real dt = 0.001;
        const long steps = 3000;

        rigid_body_params p;
        p.mass = 1200.0;
        p.inertia = {500.0, 1800.0, 2000.0};
        const wrench_script script = thrown_body_script(p);

        rigid_body_state s;
        s.pos = {0, 0, 1};
        s.vel = {10, 0, 5};
        s.omega = {0, 0, 0.5};

        for (long i = 0; i < steps; ++i)
        {
            if (writer)
                writer->row(rigid_body_row(i, dt, s));
            step(s, p, sample(script, i * dt), dt);
        }
        if (writer)
            writer->row(rigid_body_row(steps, dt, s));
        return s;
    }
}