#pragma once
#include "core/rigid_body.hpp"
#include <string>
#include <vector>

namespace rallysim
{
    inline std::vector<std::string> rigid_body_columns()
    {
        return {"step", "time",
                "pos.x", "pos.y", "pos.z",
                "vel.x", "vel.y", "vel.z",
                "quat.w", "quat.x", "quat.y", "quat.z",
                "omega.x", "omega.y", "omega.z"};
    }

    inline std::vector<std::string> rigid_body_units()
    {
        return {"count", "s", "m", "m", "m", "m/s", "m/s", "m/s", "-", "-", "-", "-", "rad/s", "rad/s", "rad/s"};
    }

    inline std::vector<Real> rigid_body_row(long step, Real dt, const rigid_body_state &s)
    {
        return {static_cast<Real>(step), step * dt,
                s.pos.x, s.pos.y, s.pos.z,
                s.vel.x, s.vel.y, s.vel.z,
                s.q.w, s.q.x, s.q.y, s.q.z,
                s.omega.x, s.omega.y, s.omega.z};
    }
}