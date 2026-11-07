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
        return {"omega", "slipRatio", "slipAngle", "Fz", "Fx", "Fy", "driveTorque", "brakeTorque",
                "deflection", "deflVel", "springF", "damperF"};
    }
    inline std::vector<std::string> wheel_units()
    {
        return {"rad/s", "-", "rad", "N", "N", "N", "N m", "N m", "m", "m/s", "N", "N"};
    }

    inline std::vector<std::string> four_wheel_columns()
    {
        std::vector<std::string> c{"step", "time", "pos.x", "pos.y", "yaw", "speed",
                                   "steer", "throttle", "brake", "handbrake", "u", "v", "yaw_rate", "beta", "a_lat",
                                   "pos.z", "roll", "pitch"};
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
                {"drive_split_front", p.drive_brake.drive_split_front},
                {"springs", p.vertical == vertical_model::springs ? 1.0 : 0.0},
                {"spring_front", p.front_corner.spring_rate}, {"damper_front", p.front_corner.damper_rate},
                {"spring_rear", p.rear_corner.spring_rate}, {"damper_rear", p.rear_corner.damper_rate}};
    }

    inline std::vector<std::string> four_wheel_units()
    {
        std::vector<std::string> u{"count", "s", "m", "m", "rad", "m/s",
                                   "rad", "0-1", "0-1", "0-1", "m/s", "m/s", "rad/s", "rad", "m/s^2",
                                   "m", "rad", "rad"};
        for (std::size_t i = 0; i < kWheels; ++i)
            for (const std::string &unit : wheel_units())
                u.push_back(unit);
        return u;
    }

    // `input` is the command applied at this step (the pedals as given, the steer limited); everything after it
    // is computed by evaluate(), the same function the step uses. slipRatio and slipAngle are the steady
    // (kinematic) slips of the contact patch, from the wheel's spin and the patch velocity: -1 is a locked wheel
    // sliding forward. The relaxed slips inside the wheel are capped at the grip, so they say how far the tyre is
    // twisted, not how fast it slides. Fx and Fy are in the wheel's frame; driveTorque and brakeTorque are what
    // the pedals put on that wheel. deflection, deflVel, springF and damperF are the corner's suspension (zero in
    // the planar model). pos.z is the CG's height; roll and pitch follow ISO 8855 (positive: right side down,
    // nose down). See docs/telemetry_schema.md.
    inline std::vector<Real> four_wheel_row(long step, Real dt, const four_wheel_state &s,
                                            const four_wheel_params &p, const four_wheel_input &input)
    {
        const four_wheel_forces f = evaluate(s, p, input);
        const Real speed = std::hypot(f.vel.u, f.vel.v);
        std::vector<Real> row{static_cast<Real>(step), step * dt, s.body.pos.x, s.body.pos.y, yaw_angle(s.body.q), speed,
                              f.steer, input.throttle, input.brake, input.handbrake,
                              f.vel.u, f.vel.v, f.vel.r, std::atan2(f.vel.v, f.vel.u),
                              speed * f.vel.r, s.body.pos.z, roll_angle(s.body.q), pitch_angle(s.body.q)};
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            const wheel_state &w = s.wheels[i];
            row.insert(row.end(), {w.omega, steady_slip_ratio(p.wheel, w.omega, f.patch[i].vx),
                                   std::atan(steady_lateral_slip(p.wheel, w.omega, f.patch[i])), f.fz[i], f.tyre[i].fx,
                                   f.tyre[i].fy, f.torque[i].drive, f.torque[i].brake,
                                   f.corner[i].deflection, f.corner[i].deflection_rate, f.corner[i].spring,
                                   f.corner[i].damper});
        }
        return row;
    }
}
