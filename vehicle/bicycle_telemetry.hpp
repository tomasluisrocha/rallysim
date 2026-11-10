#pragma once
#include <string>
#include <vector>
#include "core/telemetry.hpp"
#include "vehicle/bicycle.hpp"

namespace rallysim
{
    inline std::vector<telemetry_column> bicycle_columns()
    {
        return {{"step", "count"}, {"time", "s"}, {"pos.x", "m"}, {"pos.y", "m"}, {"yaw", "rad"}, {"speed", "m/s"},
                {"steer", "rad"}, {"accel", "m/s^2"}, {"beta", "rad"}, {"yaw_rate", "rad/s"}, {"a_lat", "m/s^2"}};
    }

    // The car's geometry for line 1 of the file, so a viewer can draw it to scale.
    inline telemetry_meta bicycle_meta(const bicycle_params &p)
    {
        return {{"model", 1}, {"lf", p.lf}, {"lr", p.lr}, {"max_steer", p.max_steer}};
    }

    // `input` is the command applied at this step. The logged steer is the limited value the model
    // actually used. beta, yaw_rate and a_lat are derived from the state, so the model carries no
    // fields that only the logger needs. a_lat = speed * yaw_rate: the centripetal acceleration of a
    // steady turn, positive when turning left.
    inline std::vector<Real> bicycle_row(long step, Real dt, const bicycle_state &s,
                                         const bicycle_params &p, const bicycle_input &input)
    {
        const Real steer = applied_steer(p, input.steer);
        const Real r = yaw_rate(s, p, steer);
        return {static_cast<Real>(step), step * dt, s.x, s.y, s.yaw, s.v,
                steer, input.accel, sideslip(p, steer), r, s.v * r};
    }
}
