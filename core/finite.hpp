#pragma once
#include <stdexcept>
#include <string>
#include "core/vec3.hpp"

namespace rallysim
{
    // Stops a run the moment its state stops being a set of real numbers (any NaN or
    // infinity anywhere in the state). A NaN never crashes anything by itself: it spreads through the state in a step or
    // two and the run goes on producing garbage, so it is caught where it first appears, with the field, the step
    // and the time.
    //
    // Works for any state that has a non_finite_field(state) next to it (rigid body, bicycle, four-wheel car),
    // returning the name of the first bad field or nullptr. Called by the scenario runners after every step, not
    // inside step() itself, which stays the hot loop. `step` is the step count the state belongs to (0 is the
    // initial state).
    template <class State>
    inline void require_finite(const State &s, long step, Real dt)
    {
        if (const char *bad = non_finite_field(s))
            throw std::runtime_error(std::string("non-finite state: ") + bad + " at step " + std::to_string(step) +
                                     " (t = " + std::to_string(step * dt) + " s)");
    }
}
