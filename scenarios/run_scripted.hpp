#pragma once
#include "core/rigid_body.hpp"
#include "core/rigid_body_telemetry.hpp"
#include "core/telemetry.hpp"
#include "driver/scripts.hpp"

namespace rallysim
{
    // Everything a scripted rigid-body scenario needs.
    struct scripted_run
    {
        rigid_body_params params;
        rigid_body_state initial;
        wrench_script script;
        Real dt = 0.001; // s, fixed step
        long steps = 0;
    };

    // Runs the scenario with a fixed step and returns the final state.
    // If writer is not null, one telemetry row is written per step, plus one for the final state.
    inline rigid_body_state run(const scripted_run &r, telemetry_writer *writer = nullptr)
    {
        rigid_body_state s = r.initial;
        for (long i = 0; i < r.steps; ++i)
        {
            const wrench w = sample(r.script, i * r.dt);
            if (writer)
                writer->row(rigid_body_row(i, r.dt, s, w));
            step(s, r.params, w, r.dt);
        }
        if (writer)
            writer->row(rigid_body_row(r.steps, r.dt, s, sample(r.script, r.steps * r.dt)));
        return s;
    }
}
