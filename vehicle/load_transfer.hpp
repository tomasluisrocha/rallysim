#pragma once
#include <algorithm>
#include "core/rigid_body.hpp"

namespace rallysim
{
    // Quasi-static load transfer, shared by the single-track car and the four-wheel car's planar model (the springs
    // model gets its loads from the springs instead). Plain numbers in, so any model can use it.

    // The axle loads (N) of a car of `mass` (kg) whose CG is lf behind the front axle, lr ahead of the rear one and
    // cg_height above the ground, under a longitudinal acceleration a_x (m/s^2): the static split weight lr / L and
    // weight lf / L, then
    // m a_x h / L moved from the front to the rear (accelerating) or back (braking). The two always add up to the
    // weight: the transfer stops when one axle carries nothing (a wheelie or a stoppie).
    inline void longitudinal_axle_loads(Real mass, Real lf, Real lr, Real cg_height, Real a_x, Real &front, Real &rear)
    {
        const Real weight = mass * kGravity, L = lf + lr;
        const Real front_static = weight * lr / L, rear_static = weight * lf / L;
        const Real shift = std::clamp(mass * a_x * cg_height / L, -rear_static, front_static);
        front = front_static - shift;
        rear = rear_static + shift;
    }

    // Splits one axle's load between its left and right wheel. `transfer` is the load moved from the left wheel to
    // the right one; it is limited so that neither wheel goes below zero: the inner wheel lifts and the outer one
    // carries the whole axle.
    inline void split_axle(Real axle, Real transfer, Real &left, Real &right)
    {
        const Real half = std::max(axle, static_cast<Real>(0)) / 2;
        const Real shift = std::clamp(transfer, -half, half);
        left = half - shift;
        right = half + shift;
    }
}
