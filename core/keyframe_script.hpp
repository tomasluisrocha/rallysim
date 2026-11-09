#pragma once
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <iterator>
#include <vector>
#include "core/vec3.hpp"

namespace rallysim
{
    // A value (driver commands, a force and torque, or one number of a scenario-file timeline) that applies at time
    // t (s).
    template <class T>
    struct keyframe
    {
        Real t;
        T value;
    };

    // Blend of two numbers: a at f = 0, b at f = 1. The input types' lerps blend each of their numbers with it.
    inline Real lerp(Real a, Real b, Real f)
    {
        const Real g = static_cast<Real>(1) - f;
        return g * a + f * b;
    }

    // A list of keyframes with strictly increasing times, sampled with linear interpolation.
    // T needs a function lerp(const T &a, const T &b, Real f) next to its definition, giving a at f = 0
    // and b at f = 1; Real uses the one above, each input type defines its own (bicycle_input, wrench, ...).
    template <class T>
    struct keyframe_script
    {
        std::vector<keyframe<T>> keys;
    };

    template <class T>
    bool is_valid(const keyframe_script<T> &script)
    {
        if (script.keys.empty())
            return false;
        for (std::size_t i = 1; i < script.keys.size(); ++i)
            if (!(script.keys[i].t > script.keys[i - 1].t))
                return false;
        return true;
    }

    // The value at time t. Linear between keyframes; holds the first value before the first keyframe
    // and the last value after the last one. Exactly on a keyframe it returns that keyframe's value.
    template <class T>
    T sample(const keyframe_script<T> &script, Real t)
    {
        assert(is_valid(script));
        const auto &k = script.keys;

        if (t <= k.front().t)
            return k.front().value;
        if (t >= k.back().t)
            return k.back().value;

        // first keyframe at or after t; it cannot be the first one, because t is past the front
        auto it = std::lower_bound(k.begin(), k.end(), t,
                                   [](const keyframe<T> &kf, Real val)
                                   { return kf.t < val; });

        const std::size_t i = static_cast<std::size_t>(std::distance(k.begin(), it));
        const Real f = (t - k[i - 1].t) / (k[i].t - k[i - 1].t); // 0 at the earlier key, 1 at the later
        return lerp(k[i - 1].value, k[i].value, f);
    }
}
