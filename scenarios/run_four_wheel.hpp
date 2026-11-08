#pragma once
#include "core/finite.hpp"
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
        Real hold_speed = 0; // m/s; if above zero, the speed controller replaces the script's throttle and brake
        Real dt = 0.001;     // s, fixed step
        long steps = 0;
    };

    // The commands for time t: the script, with the throttle and brake replaced by the speed controller
    // (driver/speed_hold.hpp) when hold_speed is set. The steer and the handbrake always come from the script.
    // While the handbrake is up the controller lifts off the throttle, as a driver does (clutch in): otherwise
    // the drive torque on the rear wheels would fight the handbrake.
    inline four_wheel_input driver_input(const four_wheel_run &r, const four_wheel_state &s, Real t)
    {
        four_wheel_input in = sample(r.script, t);
        if (r.hold_speed > 0 && in.handbrake > 0)
        {
            in.throttle = 0;
            in.brake = 0;
        }
        else if (r.hold_speed > 0)
        {
            const drive_brake_params &d = r.params.drive_brake;
            const Real R = r.params.wheel.radius;
            const pedals p = speed_hold_pedals(r.params.chassis.mass, r.hold_speed, velocity_in_body(s.body).u,
                                               d.max_drive_torque / R, d.max_brake_torque / R);
            in.throttle = p.throttle;
            in.brake = p.brake;
        }
        return in;
    }

    // Runs the scenario with a fixed step and returns the final state.
    // If writer is not null, one telemetry row is written per step, plus one for the final state.
    // Stops with an exception naming the field, step and time if the state stops being finite (core/finite.hpp).
    // Energy books (vehicle/four_wheel_energy.hpp) are kept when telemetry is written (its energy columns) or when
    // `books` is given, which then holds the whole run's on return. Keeping them costs one more evaluate() per step,
    // so a plain run does not.
    inline four_wheel_state run(const four_wheel_run &r, telemetry_writer *writer = nullptr, energy_books *books = nullptr)
    {
        four_wheel_state s = r.initial;
        require_finite(s, 0, r.dt);
        energy_books kept(s, r.params);
        const bool keep_books = writer || books;
        for (long i = 0; i < r.steps; ++i)
        {
            const four_wheel_input in = driver_input(r, s, i * r.dt);
            if (writer)
                writer->row(four_wheel_row(i, r.dt, s, r.params, in, kept));
            if (keep_books)
                kept.add(rates_of(s, r.params, evaluate(s, r.params, in)), r.dt); // this step's work, before it
            step(s, r.params, in, r.dt);
            require_finite(s, i + 1, r.dt);
        }
        if (writer)
            writer->row(four_wheel_row(r.steps, r.dt, s, r.params, driver_input(r, s, r.steps * r.dt), kept));
        if (books)
            *books = kept;
        return s;
    }
}
