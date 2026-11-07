#pragma once
#include <cassert>
#include <cmath>
#include <algorithm>
#include "core/vec3.hpp"

namespace rallysim
{
    struct bicycle_state
    {
        // x,y are the center of gravity
        Real x = 0, y = 0, yaw = 0, v = 0;
    };

    struct bicycle_params
    {
        // distance from front and rear wheel to center of gravity (m)
        Real lf = 1.3, lr = 1.2;

        // largest front wheel angle (rad); the steering input is clamped to +-max_steer
        Real max_steer = 0.6;

        // Returns wheelbase, as the distance between front and rear wheel
        Real wheelbase() const
        {
            return (lr + lf);
        }
    };

    struct bicycle_input
    {
        // (radians, positive = left) (m/s**2 alongside heading)
        Real steer = 0, accel = 0;
    };

    inline void step(bicycle_state &state, const bicycle_params &params, const bicycle_input &input, Real dt)
    {
        const Real L = params.wheelbase();
        assert(L > 0);
        const Real steer = std::clamp(input.steer, -params.max_steer, params.max_steer);
        // speed first
        state.v += input.accel * dt;
        // angle between the heading and the velocity of the CG (sideslip)
        const Real beta = std::atan((params.lr / L) * std::tan(steer));
        const Real yaw_rate = state.v * std::cos(beta) * std::tan(steer) / L;
        // heading half way through the step: more accurate on a curve than the start heading
        const Real mid = state.yaw + 0.5 * yaw_rate * dt;
        state.x += state.v * std::cos(mid + beta) * dt;
        state.y += state.v * std::sin(mid + beta) * dt;
        state.yaw += yaw_rate * dt;
    }

}