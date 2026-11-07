#pragma once
#include "core/telemetry.hpp"
#include "driver/scripts.hpp"
#include "driver/speed_hold.hpp"
#include "vehicle/four_wheel.hpp"
#include "vehicle/four_wheel_telemetry.hpp"

namespace rallysim
{
    // Everything a scripted four-wheel scenario needs.
    struct four_wheel_run
    {
        four_wheel_params params;
        four_wheel_state initial;
        four_wheel_script script;
        Real hold_speed = 0; // m/s; if above zero, the speed controller replaces the script's drive torque
        Real dt = 0.001;     // s, fixed step
        long steps = 0;
    };

    // The commands for time t: the script, with the drive torque replaced by the speed controller
    // (driver/speed_hold.hpp) when hold_speed is set. The controller asks for a force; the rear wheels get
    // that force times the wheel radius as torque.
    inline four_wheel_input driver_input(const four_wheel_run &r, const four_wheel_state &s, Real t)
    {
        four_wheel_input in = sample(r.script, t);
        if (r.hold_speed > 0)
            in.drive_torque = speed_hold_force(r.params.chassis.mass, r.hold_speed, velocity_in_body(s.body).u) *
                              r.params.wheel.radius;
        return in;
    }

    // Runs the scenario with a fixed step and returns the final state.
    // If writer is not null, one telemetry row is written per step, plus one for the final state.
    inline four_wheel_state run(const four_wheel_run &r, telemetry_writer *writer = nullptr)
    {
        four_wheel_state s = r.initial;
        for (long i = 0; i < r.steps; ++i)
        {
            const four_wheel_input in = driver_input(r, s, i * r.dt);
            if (writer)
                writer->row(four_wheel_row(i, r.dt, s, r.params, in));
            step(s, r.params, in, r.dt);
        }
        if (writer)
            writer->row(four_wheel_row(r.steps, r.dt, s, r.params, driver_input(r, s, r.steps * r.dt)));
        return s;
    }
}
