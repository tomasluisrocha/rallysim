#pragma once
#include "core/rigid_body.hpp"
#include "core/rigid_body_telemetry.hpp"
#include "core/telemetry.hpp"
#include "core/wrench_script.hpp"

namespace rallysim
{
        inline wrench_script driven_body_script(const rigid_body_params &p)
    {
        const Real lift = p.mass * kGravity;
        auto key = [&](Real t, Real fy)
        { return keyframe{t, wrench{vec3{0, fy, lift}, vec3{}}}; };

        wrench_script s;
        s.keys = {key(0.0, 0),
                  key(1.0, 0),      // straight
                  key(2.0, 1500),   // ramp up: push left
                  key(4.0, 1500),   // hold
                  key(5.0, -1500),  // swing to push right
                  key(7.0, -1500),  // hold
                  key(8.0, 0),      // ramp off
                  key(9.0, 0)};     // straight
        return s;
    }

    // runs the driven-body scenario and returns the final state
    inline rigid_body_state run_driven_body(telemetry_writer *writer = nullptr)
    {
        const Real dt = 0.001;
        const long steps = 9000;

        rigid_body_params p;
        p.mass = 1200.0;
        p.inertia = {500.0, 1800.0, 2000.0};
        const wrench_script script = driven_body_script(p);

        rigid_body_state s;
        s.vel = {5, 0, 0};

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