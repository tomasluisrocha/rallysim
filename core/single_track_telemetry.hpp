#pragma once
#include <cmath>
#include <string>
#include <vector>
#include "core/single_track.hpp"

namespace rallysim
{
    inline std::vector<std::string> single_track_columns()
    {
        return {"step", "time", "pos.x", "pos.y", "yaw", "speed",
                "steer", "drive_force", "u", "v", "yaw_rate", "beta", "a_lat",
                "axle.front.slipAngle", "axle.rear.slipAngle", "axle.front.Fy", "axle.rear.Fy",
                "wheel.FL.Fz", "wheel.FR.Fz", "wheel.RL.Fz", "wheel.RR.Fz",
                "axle.front.grip", "axle.rear.grip", "axle.rear.Fx"};
    }

    inline std::vector<std::string> single_track_units()
    {
        return {"count", "s", "m", "m", "rad", "m/s",
                "rad", "N", "m/s", "m/s", "rad/s", "rad", "m/s^2",
                "rad", "rad", "N", "N",
                "N", "N", "N", "N",
                "N", "N", "N"};
    }

    // `input` is the command applied at this step. Everything after it is computed by evaluate(), the
    // same function the step uses, so the model carries no fields that only the logger needs. The logged
    // steer is the limited value the model actually used; drive_force is the command, axle.rear.Fx what
    // the rear axle passed to the road. speed is the length of the velocity of the CG; beta is its angle
    // to the heading; a_lat = speed * yaw_rate, positive when turning left. See docs/telemetry_schema.md.
    inline std::vector<Real> single_track_row(long step, Real dt, const rigid_body_state &s,
                                              const single_track_params &p, const single_track_input &input)
    {
        const single_track_forces f = evaluate(s, p, input);
        const body_velocity &vel = f.vel;
        const Real speed = std::hypot(vel.u, vel.v);
        return {static_cast<Real>(step), step * dt, s.pos.x, s.pos.y, yaw_angle(s.q), speed,
                f.steer, input.drive_force, vel.u, vel.v, vel.r, std::atan2(vel.v, vel.u), speed * vel.r,
                f.slip.front, f.slip.rear, f.fy.front, f.fy.rear,
                f.fz.front_left, f.fz.front_right, f.fz.rear_left, f.fz.rear_right,
                f.grip.front, f.grip.rear, f.fx_rear};
    }
}
