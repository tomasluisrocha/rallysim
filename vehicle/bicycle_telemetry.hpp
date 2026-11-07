#pragma once
#include <string>
#include <vector>
#include "core/telemetry.hpp"
#include "vehicle/bicycle.hpp"

namespace rallysim
{
    inline std::vector<std::string> bicycle_columns()
    {
        return {"step", "time", "pos.x", "pos.y", "yaw", "speed",
                "steer", "accel", "beta", "yaw_rate", "a_lat"};
    }

    // The car's geometry for line 1 of the file, so a viewer can draw it to scale.
    inline telemetry_meta bicycle_meta(const bicycle_params &p)
    {
        return {{"model", 1}, {"lf", p.lf}, {"lr", p.lr}, {"max_steer", p.max_steer}};
    }

    inline std::vector<std::string> bicycle_units()
    {
        return {"count", "s", "m", "m", "rad", "m/s",
                "rad", "m/s^2", "rad", "rad/s", "m/s^2"};
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
