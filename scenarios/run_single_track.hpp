#pragma once
#include "core/single_track.hpp"
#include "core/single_track_script.hpp"
#include "core/single_track_telemetry.hpp"
#include "core/telemetry.hpp"

namespace rallysim
{
    // Everything a scripted single-track scenario needs.
    struct single_track_run
    {
        single_track_params params;
        rigid_body_state initial;
        single_track_script script;
        Real hold_speed = 0; // m/s; if above zero, a speed controller replaces the script's drive force
        Real dt = 0.001;     // s, fixed step
        long steps = 0;
    };

    // Gain of the speed controller (1/s): drive force = mass * gain * speed error.
    inline constexpr Real kSpeedHoldGain = 10.0;

    // A car on the ground plane, pointing along +x and moving forward at `speed`.
    inline rigid_body_state single_track_initial(Real speed)
    {
        rigid_body_state s;
        s.vel = {speed, 0, 0};
        return s;
    }

    // The commands for time t: the script, with the drive force replaced by a proportional
    // speed controller on the forward speed when hold_speed is set. Cruise control, not part of the model.
    inline single_track_input driver_input(const single_track_run &r, const rigid_body_state &s, Real t)
    {
        single_track_input in = sample(r.script, t);
        if (r.hold_speed > 0)
            in.drive_force = r.params.chassis.mass * kSpeedHoldGain * (r.hold_speed - velocity_in_body(s).u);
        return in;
    }

    // Runs the scenario with a fixed step and returns the final state.
    // If writer is not null, one telemetry row is written per step, plus one for the final state.
    inline rigid_body_state run(const single_track_run &r, telemetry_writer *writer = nullptr)
    {
        rigid_body_state s = r.initial;
        for (long i = 0; i < r.steps; ++i)
        {
            const single_track_input in = driver_input(r, s, i * r.dt);
            if (writer)
                writer->row(single_track_row(i, r.dt, s, r.params, in));
            step(s, r.params, in, r.dt);
        }
        if (writer)
            writer->row(single_track_row(r.steps, r.dt, s, r.params, driver_input(r, s, r.steps * r.dt)));
        return s;
    }
}