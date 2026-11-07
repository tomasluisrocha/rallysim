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
                "alpha_f", "alpha_r", "fy_f", "fy_r"};
    }

    inline std::vector<std::string> single_track_units()
    {
        return {"count", "s", "m", "m", "rad", "m/s",
                "rad", "N", "m/s", "m/s", "rad/s", "rad", "m/s^2",
                "rad", "rad", "N", "N"};
    }

    // `input` is the command applied at this step. The logged steer is the limited value the model
    // actually used. Everything else is derived from the state with the same functions as the step,
    // so the model carries no fields that only the logger needs. speed is the length of the velocity
    // of the CG; beta is its angle to the heading; a_lat = speed * yaw_rate, positive when turning left.
    // alpha_f and alpha_r are the slip angles and fy_f and fy_r the lateral forces of the front and
    // rear axle, in the axle frame (sideways to the wheels).
    inline std::vector<Real> single_track_row(long step, Real dt, const rigid_body_state &s,
                                              const single_track_params &p, const single_track_input &input)
    {
        const body_velocity vel = velocity_in_body(s);
        const Real steer = applied_steer(p, input.steer);
        const axle_pair slip = slip_angles(p, vel, steer);
        const axle_pair fy = lateral_forces(p, slip, axle_loads(p));
        const Real speed = std::hypot(vel.u, vel.v);
        return {static_cast<Real>(step), step * dt, s.pos.x, s.pos.y, yaw_angle(s.q), speed,
                steer, input.drive_force, vel.u, vel.v, vel.r, std::atan2(vel.v, vel.u), speed * vel.r,
                slip.front, slip.rear, fy.front, fy.rear};
    }
}