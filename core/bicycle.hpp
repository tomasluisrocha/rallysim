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

    // The front wheel angle the model actually uses: the input, limited to +-max_steer.
    inline Real applied_steer(const bicycle_params &params, Real steer)
    {
        return std::clamp(steer, -params.max_steer, params.max_steer);
    }

    // Angle between the heading and the velocity of the CG (sideslip), for an already limited steer.
    inline Real sideslip(const bicycle_params &params, Real steer)
    {
        return std::atan(params.lr / params.wheelbase() * std::tan(steer));
    }

    // Yaw rate (rad/s) for an already limited steer, at the speed in `state`.
    inline Real yaw_rate(const bicycle_state &state, const bicycle_params &params, Real steer)
    {
        return state.v * std::cos(sideslip(params, steer)) * std::tan(steer) / params.wheelbase();
    }

    inline void step(bicycle_state &state, const bicycle_params &params, const bicycle_input &input, Real dt)
    {
        assert(params.wheelbase() > 0);
        const Real steer = applied_steer(params, input.steer);
        // speed first
        state.v += input.accel * dt;
        const Real beta = sideslip(params, steer);
        const Real r = yaw_rate(state, params, steer);
        // heading half way through the step: more accurate on a curve than the start heading
        const Real mid = state.yaw + 0.5 * r * dt;
        state.x += state.v * std::cos(mid + beta) * dt;
        state.y += state.v * std::sin(mid + beta) * dt;
        state.yaw += r * dt;
    }

}
