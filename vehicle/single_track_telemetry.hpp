#pragma once
#include <cmath>
#include <string>
#include <vector>
#include "core/telemetry.hpp"
#include "vehicle/single_track.hpp"

namespace rallysim
{
    inline std::vector<telemetry_column> single_track_columns()
    {
        return {{"step", "count"}, {"time", "s"}, {"pos.x", "m"}, {"pos.y", "m"}, {"yaw", "rad"}, {"speed", "m/s"},
                {"steer", "rad"}, {"drive_force", "N"}, {"u", "m/s"}, {"v", "m/s"}, {"yaw_rate", "rad/s"},
                {"beta", "rad"}, {"a_lat", "m/s^2"},
                {"axle.front.slipAngle", "rad"}, {"axle.rear.slipAngle", "rad"},
                {"axle.front.Fy", "N"}, {"axle.rear.Fy", "N"},
                {"wheel.FL.Fz", "N"}, {"wheel.FR.Fz", "N"}, {"wheel.RL.Fz", "N"}, {"wheel.RR.Fz", "N"},
                {"axle.front.grip", "N"}, {"axle.rear.grip", "N"}, {"axle.rear.Fx", "N"}};
    }

    // The car's geometry and grip for line 1 of the file, so a viewer can draw it to scale.
    inline telemetry_meta single_track_meta(const single_track_params &p)
    {
        return {{"model", 2}, {"lf", p.lf}, {"lr", p.lr}, {"track_front", p.track_width}, {"track_rear", p.track_width},
                {"cg_height", p.cg_height}, {"max_steer", p.max_steer}, {"mass", p.chassis.mass}};
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
