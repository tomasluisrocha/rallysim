#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include "core/vec3.hpp"

namespace rallysim
{
    // Placeholder drivetrain and brakes (phase 4, step 4): turns pedal positions into a drive torque and a brake
    // torque on each wheel, from a few car characteristics. Phase 7 replaces the drive side with an engine,
    // clutch, gearbox and differentials, and the brake side with a hydraulic model; the pedals stay the same.
    //
    // Wheel order as everywhere: front-left, front-right, rear-left, rear-right. Torques are at the wheel.
    struct drive_brake_params
    {
        Real max_drive_torque = 6000;     // N m at the wheels at full throttle, all wheels together: an engine in a low gear
        Real max_drive_power = 200000;    // W at full throttle (about a Rally2 engine): above P / T_max the torque is P / omega
        Real drive_split_front = 0;       // share of the drive torque on the front axle: 0 RWD, 1 FWD, 0.5 AWD
        Real max_brake_torque = 6000;     // N m at full brake, all wheels together: enough to lock all four
        Real brake_bias_front = 0.70;     // share of the brake torque on the front axle. At or above the front's share
                                          // of the load while braking at mu g (0.68 for this car at mu = 1), so the
                                          // fronts lock before the rears; about 0.6 would suit gravel (mu 0.6)
        Real max_handbrake_torque = 3000; // N m at full handbrake, on the two rear wheels: enough to lock them
    };

    // Pedal positions, each 0 (released) to 1 (fully pressed). Values outside are clamped.
    struct pedals
    {
        Real throttle = 0, brake = 0, handbrake = 0;
    };

    // Torques on one wheel (N m): drive (+forward) and brake (a magnitude; it always opposes the wheel's spin).
    struct wheel_torque
    {
        Real drive = 0, brake = 0;
    };

    inline Real pedal(Real position)
    {
        return std::clamp(position, static_cast<Real>(0), static_cast<Real>(1));
    }

    // Drive torque available at full throttle (N m, all driven wheels together) when the driven wheels spin at
    // an average of omega (rad/s): the torque limit at low speed, the power limit P / omega above P / T_max.
    // With an open differential the engine turns with the average of the driven wheels, so a spinning wheel
    // runs out of power instead of speeding up for ever.
    inline Real available_drive_torque(const drive_brake_params &p, Real omega)
    {
        return std::min(p.max_drive_torque, p.max_drive_power / std::max(omega, p.max_drive_power / p.max_drive_torque));
    }

    // Each axle's share is split equally between its two wheels (an open differential, as far as torque goes).
    // The handbrake acts on the rear wheels only. `omega` is each wheel's spin (rad/s), for the power limit.
    inline std::array<wheel_torque, 4> wheel_torques(const drive_brake_params &p, const pedals &in,
                                                     const std::array<Real, 4> &omega)
    {
        const Real driven_omega = p.drive_split_front * 0.5 * (omega[0] + omega[1]) +
                                  (1 - p.drive_split_front) * 0.5 * (omega[2] + omega[3]);
        const Real drive = pedal(in.throttle) * available_drive_torque(p, driven_omega);
        const Real brake = pedal(in.brake) * p.max_brake_torque;
        const Real handbrake = pedal(in.handbrake) * p.max_handbrake_torque;

        const wheel_torque front{0.5 * p.drive_split_front * drive, 0.5 * p.brake_bias_front * brake};
        const wheel_torque rear{0.5 * (1 - p.drive_split_front) * drive, 0.5 * ((1 - p.brake_bias_front) * brake + handbrake)};
        return {front, front, rear, rear};
    }
}
