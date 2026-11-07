#pragma once
#include <cmath>
#include <string>
#include <vector>
#include "core/telemetry.hpp"
#include "vehicle/four_wheel.hpp"

namespace rallysim
{
    inline const char *wheel_label(std::size_t i)
    {
        static const char *const labels[kWheels] = {"FL", "FR", "RL", "RR"};
        return labels[i];
    }

    // The per-wheel columns, in this order for each wheel (book section 11.1 names).
    inline std::vector<std::string> wheel_fields()
    {
        return {"omega", "slipRatio", "slipAngle", "Fz", "Fx", "Fy", "driveTorque", "brakeTorque"};
    }
    inline std::vector<std::string> wheel_units() { return {"rad/s", "-", "rad", "N", "N", "N", "N m", "N m"}; }

    inline std::vector<std::string> four_wheel_columns()
    {
        std::vector<std::string> c{"step", "time", "pos.x", "pos.y", "yaw", "speed",
                                   "steer", "throttle", "brake", "handbrake", "u", "v", "yaw_rate", "beta", "a_lat"};
        for (std::size_t i = 0; i < kWheels; ++i)
            for (const std::string &field : wheel_fields())
                c.push_back(std::string("wheel.") + wheel_label(i) + "." + field);
        return c;
    }

    // The car's geometry and tyre grip for line 1 of the file, so a viewer can draw it to scale and work out how
    // much of each tyre's grip is in use: grip = mu (1 - k (Fz - Fz0) / Fz0) Fz.
    inline telemetry_meta four_wheel_meta(const four_wheel_params &p)
    {
        return {{"model", 4}, {"lf", p.lf}, {"lr", p.lr}, {"track_front", p.track_front}, {"track_rear", p.track_rear},
                {"cg_height", p.cg_height}, {"max_steer", p.max_steer}, {"mass", p.chassis.mass},
                {"wheel_radius", p.wheel.radius},
                {"mu_front", p.front_tyre.friction_coefficient}, {"k_front", p.front_tyre.load_sensitivity},
                {"fz0_front", p.front_tyre.nominal_load},
                {"mu_rear", p.rear_tyre.friction_coefficient}, {"k_rear", p.rear_tyre.load_sensitivity},
                {"fz0_rear", p.rear_tyre.nominal_load},
                {"drive_split_front", p.drive_brake.drive_split_front}};
    }

    inline std::vector<std::string> four_wheel_units()
    {
        std::vector<std::string> u{"count", "s", "m", "m", "rad", "m/s",
                                   "rad", "0-1", "0-1", "0-1", "m/s", "m/s", "rad/s", "rad", "m/s^2"};
        for (std::size_t i = 0; i < kWheels; ++i)
            for (const std::string &unit : wheel_units())
                u.push_back(unit);
        return u;
    }

    // `input` is the command applied at this step (the pedals as given, the steer limited); everything after it
    // is computed by evaluate(), the same function the step uses. slipRatio and slipAngle are the wheel's relaxed
    // slips (slipAngle = atan of the lateral slip); Fx and Fy are in the wheel's frame; driveTorque and
    // brakeTorque are what the pedals put on that wheel. See docs/telemetry_schema.md.
    inline std::vector<Real> four_wheel_row(long step, Real dt, const four_wheel_state &s,
                                            const four_wheel_params &p, const four_wheel_input &input)
    {
        const four_wheel_forces f = evaluate(s, p, input);
        const Real speed = std::hypot(f.vel.u, f.vel.v);
        std::vector<Real> row{static_cast<Real>(step), step * dt, s.body.pos.x, s.body.pos.y, yaw_angle(s.body.q), speed,
                              f.steer, input.throttle, input.brake, input.handbrake,
                              f.vel.u, f.vel.v, f.vel.r, std::atan2(f.vel.v, f.vel.u),
                              speed * f.vel.r};
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            const wheel_state &w = s.wheels[i];
            row.insert(row.end(), {w.omega, w.slip_ratio, std::atan(w.lateral_slip), f.fz[i], f.tyre[i].fx, f.tyre[i].fy,
                                   f.torque[i].drive, f.torque[i].brake});
        }
        return row;
    }
}
