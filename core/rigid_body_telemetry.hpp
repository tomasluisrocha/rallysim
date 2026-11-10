#pragma once
#include <string>
#include <vector>
#include "core/rigid_body.hpp"
#include "core/telemetry.hpp"

namespace rallysim
{
    inline std::vector<telemetry_column> rigid_body_columns()
    {
        return {{"step", "count"}, {"time", "s"},
                {"pos.x", "m"}, {"pos.y", "m"}, {"pos.z", "m"},
                {"vel.x", "m/s"}, {"vel.y", "m/s"}, {"vel.z", "m/s"},
                {"quat.w", "-"}, {"quat.x", "-"}, {"quat.y", "-"}, {"quat.z", "-"},
                {"omega.x", "rad/s"}, {"omega.y", "rad/s"}, {"omega.z", "rad/s"},
                {"force.x", "N"}, {"force.y", "N"}, {"force.z", "N"},
                {"torque.x", "N m"}, {"torque.y", "N m"}, {"torque.z", "N m"}};
    }

    // w is the force (world frame) and torque (body frame) applied at this step.
    inline std::vector<Real> rigid_body_row(long step, Real dt, const rigid_body_state &s, const wrench &w)
    {
        return {static_cast<Real>(step), step * dt,
                s.pos.x, s.pos.y, s.pos.z,
                s.vel.x, s.vel.y, s.vel.z,
                s.q.w, s.q.x, s.q.y, s.q.z,
                s.omega.x, s.omega.y, s.omega.z,
                w.force.x, w.force.y, w.force.z,
                w.torque.x, w.torque.y, w.torque.z};
    }
}
