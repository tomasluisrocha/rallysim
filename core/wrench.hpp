#pragma once
#include "core/quat.hpp"

namespace rallysim
{
    // Net force and torque on a body at one instant.
    // Forces are collected with add_force_at / add_torque and then passed to step().
    struct wrench
    {
        vec3 force;  // N, world frame (gravity is added in step)
        vec3 torque; // N m, body frame, about the centre of mass
    };

    // Blend of two wrenches for keyframe scripts: a at f = 0, b at f = 1.
    inline wrench lerp(const wrench &a, const wrench &b, Real f)
    {
        const Real g = static_cast<Real>(1) - f;
        return {g * a.force + f * b.force, g * a.torque + f * b.torque};
    }

    // Adds a force (world frame) applied at a point (body frame, relative to the centre of mass).
    // A force away from the centre of mass also adds a torque.
    // q is the body's orientation (body -> world).
    inline void add_force_at(wrench &w, const quat &q, const vec3 &force_world, const vec3 &point_body)
    {
        w.force += force_world;
        w.torque += cross(point_body, to_body(q, force_world));
    }

    // Adds a pure torque (body frame), for example an engine reaction.
    inline void add_torque(wrench &w, const vec3 &torque_body)
    {
        w.torque += torque_body;
    }
}
