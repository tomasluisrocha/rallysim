#pragma once
#include "core/vec3.hpp"

namespace rallysim
{
    // Gain of the speed controller (1/s): drive force = mass * gain * speed error.
    inline constexpr Real kSpeedHoldGain = 10.0;

    // Cruise control, not part of any car model: a proportional controller on the forward speed (m/s)
    // that returns the drive force (N) to ask for. It settles a little below the target when something
    // pulls the car back, as a proportional controller does.
    inline Real speed_hold_force(Real mass, Real target_speed, Real forward_speed)
    {
        return mass * kSpeedHoldGain * (target_speed - forward_speed);
    }
}
