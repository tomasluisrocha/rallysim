#pragma once
#include <algorithm>
#include "core/vec3.hpp"
#include "vehicle/drive_brake.hpp"

namespace rallysim
{
    // Gain of the speed controller (1/s): drive force = mass * gain * speed error.
    inline constexpr Real kSpeedHoldGain = 10.0;

    // Cruise control, not part of any car model: a proportional controller on the forward speed (m/s)
    // that returns the drive force (N) to ask for. It settles a little below the target when something
    // pulls the car back, as a proportional controller does.
    inline Real speed_hold_force(Real mass, Real target_speed, Real forward_speed, Real gain = kSpeedHoldGain)
    {
        return mass * gain * (target_speed - forward_speed);
    }

    // The same controller with pedals: the force it asks for as throttle when positive, as brake when negative,
    // each as a fraction of the force the full pedal gives (N), clamped to 0..1.
    inline pedals speed_hold_pedals(Real mass, Real target_speed, Real forward_speed, Real full_drive_force,
                                    Real full_brake_force, Real gain = kSpeedHoldGain)
    {
        const Real force = speed_hold_force(mass, target_speed, forward_speed, gain);
        pedals p;
        if (force > 0)
            p.throttle = std::min(force / full_drive_force, static_cast<Real>(1));
        else
            p.brake = std::min(-force / full_brake_force, static_cast<Real>(1));
        return p;
    }
}
