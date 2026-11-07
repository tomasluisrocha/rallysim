#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <vector>
#include "core/wrench.hpp"

namespace rallysim
{
    // The force and torque that apply at time t (s).
    struct keyframe
    {
        Real t;
        wrench w;
    };

    // A list of keyframes with strictly increasing times.
    struct wrench_script
    {
        std::vector<keyframe> keys;
    };

    inline bool is_valid(const wrench_script &script)
    {
        if (script.keys.empty())
            return false;
        for (std::size_t i = 1; i < script.keys.size(); ++i)
            if (!(script.keys[i].t > script.keys[i - 1].t))
                return false;
        return true;
    }

    // Force and torque at time t. Linear between keyframes; holds the first value before
    // the first keyframe and the last value after the last one.
    inline wrench sample(const wrench_script &script, Real t)
    {
        assert(is_valid(script));
        const auto &k = script.keys;

        if (t <= k.front().t)
            return k.front().w;
        if (t >= k.back().t)
            return k.back().w;

        auto it = std::lower_bound(k.begin(), k.end(), t,
                                   [](const keyframe &kf, Real val)
                                   { return kf.t < val; });

        std::size_t i = std::distance(k.begin(), it);
        const Real f = (t - k[i - 1].t) / (k[i].t - k[i - 1].t);
        const Real g = static_cast<Real>(1) - f;

        return wrench{g * k[i - 1].w.force + f * k[i].w.force,
                      g * k[i - 1].w.torque + f * k[i].w.torque};
    }
}