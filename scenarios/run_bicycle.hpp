#pragma once
#include "core/bicycle.hpp"
#include "core/bicycle_telemetry.hpp"
#include "core/input_script.hpp"
#include "core/telemetry.hpp"

namespace rallysim
{
    // Everything a scripted bicycle scenario needs.
    struct bicycle_run
    {
        bicycle_params params;
        bicycle_state initial;
        input_script script;
        Real dt = 0.001; // s, fixed step
        long steps = 0;
    };

    // Runs the scenario with a fixed step and returns the final state.
    // If writer is not null, one telemetry row is written per step, plus one for the final state.
    inline bicycle_state run(const bicycle_run &r, telemetry_writer *writer = nullptr)
    {
        bicycle_state s = r.initial;
        for (long i = 0; i < r.steps; ++i)
        {
            const bicycle_input in = sample(r.script, i * r.dt);
            if (writer)
                writer->row(bicycle_row(i, r.dt, s, r.params, in));
            step(s, r.params, in, r.dt);
        }
        if (writer)
            writer->row(bicycle_row(r.steps, r.dt, s, r.params, sample(r.script, r.steps * r.dt)));
        return s;
    }
}
