#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <vector>
#include "core/bicycle.hpp"

namespace rallysim
{
    // The driver's commands that apply at time t (s).
    struct input_keyframe
    {
        Real t;
        bicycle_input in;
    };

    // A list of keyframes with strictly increasing times.
    struct input_script
    {
        std::vector<input_keyframe> keys;
    };

    inline bool is_valid(const input_script &script)
    {
        if (script.keys.empty())
            return false;
        for (std::size_t i = 1; i < script.keys.size(); ++i)
            if (!(script.keys[i].t > script.keys[i - 1].t))
                return false;
        return true;
    }

    // Steer and acceleration at time t. Linear between keyframes; holds the first value before
    // the first keyframe and the last value after the last one.
    inline bicycle_input sample(const input_script &script, Real t)
    {
        assert(is_valid(script));
        const auto &k = script.keys;

        if (t <= k.front().t)
            return k.front().in;
        if (t >= k.back().t)
            return k.back().in;

        // first keyframe at or after t; it cannot be the first one, because t is past the front
        auto it = std::lower_bound(k.begin(), k.end(), t,
                                   [](const input_keyframe &kf, Real val)
                                   { return kf.t < val; });

        const std::size_t i = std::distance(k.begin(), it);
        const Real f = (t - k[i - 1].t) / (k[i].t - k[i - 1].t); // 0 at the earlier key, 1 at the later
        const Real g = static_cast<Real>(1) - f;

        return bicycle_input{g * k[i - 1].in.steer + f * k[i].in.steer,
                             g * k[i - 1].in.accel + f * k[i].in.accel};
    }
}