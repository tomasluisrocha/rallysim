#pragma once
#include <string>
#include <utility>
#include <vector>
#include "core/finite.hpp"
#include "core/telemetry.hpp"
#include "driver/heading_hold.hpp"
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
        // The speed controller's target over time (m/s): when given, it replaces the script's throttle and brake.
        // Empty: off (the script's pedals throughout). A constant target is constant_script(v).
        keyframe_script<Real> hold_speed;
        Real speed_gain = kSpeedHoldGain; // 1/s: drive force asked per m/s below the target, per kg
        // From this time (s) the heading-hold driver (driver/heading_hold.hpp) steers towards hold_heading (rad,
        // world) instead of the script; below zero, never (the default: the script steers throughout)
        Real hold_heading_from = -1;
        Real hold_heading = 0;
        heading_hold_params heading_hold;
        Real dt = 0.001; // s, fixed step
        long steps = 0;
    };

    // Settings by name, (name, value) in the order given: rallysim --set, applied by the scenario loader.
    using settings = std::vector<std::pair<std::string, std::string>>;

    // The commands for time t: the script, with the throttle and brake replaced by the speed controller
    // (driver/speed_hold.hpp), aiming at hold_speed's target for time t, when it is given, and the steer by the
    // heading-hold driver (driver/heading_hold.hpp) from hold_heading_from on. The handbrake always comes from the
    // script.
    // While the handbrake is up the speed controller lifts off the throttle, as a driver does (clutch in):
    // otherwise the drive torque on the rear wheels would fight the handbrake.
    inline four_wheel_input driver_input(const four_wheel_run &r, const four_wheel_state &s, Real t)
    {
        four_wheel_input in = sample(r.script, t);
        if (r.hold_heading_from >= 0 && t >= r.hold_heading_from)
            in.steer = heading_hold_steer(velocity_in_body(s.body), yaw_angle(s.body.q), r.hold_heading, r.params.lf,
                                          r.params.max_steer, r.heading_hold);
        const bool holding_speed = !r.hold_speed.keys.empty();
        if (holding_speed && in.handbrake > 0)
        {
            in.throttle = 0;
            in.brake = 0;
        }
        else if (holding_speed)
        {
            const drive_brake_params &d = r.params.drive_brake;
            const Real R = r.params.wheel.radius;
            const pedals p = speed_hold_pedals(r.params.chassis.mass, sample(r.hold_speed, t), velocity_in_body(s.body).u,
                                               d.max_drive_torque / R, d.max_brake_torque / R, r.speed_gain);
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
