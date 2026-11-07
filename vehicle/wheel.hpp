#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include "vehicle/tyre.hpp"

namespace rallysim
{
    // One wheel (book section 7.1): its spin, the slip of its contact patch, and the tyre force that slip makes.
    // The car gives it the velocity of the contact patch, the vertical load and the drive and brake torques;
    // it returns the tyre force to apply to the car.
    struct wheel_params
    {
        Real radius = 0.315; // m, effective rolling radius
        Real inertia = 1.2;  // kg m^2 about the axle: tyre, rim, hub and brake disc
    };

    // What changes from step to step. The two slips are relaxed (transient) values: they follow the slip of
    // the contact patch over a short rolling distance instead of jumping to it, which is what keeps the tyre
    // well behaved at low speed and at standstill (book section 7.1).
    struct wheel_state
    {
        Real omega = 0;        // rad/s, positive rolling forward
        Real slip_ratio = 0;   // relaxed slip ratio, dimensionless
        Real lateral_slip = 0; // relaxed lateral slip, tangent of the slip angle
    };

    // Velocity of the contact patch over the ground, in the wheel frame (m/s): x along the wheel, y to its left.
    struct patch_velocity
    {
        Real vx = 0, vy = 0;
    };

    // Below this speed (m/s) the steady slips divide by it instead, so they stay finite at standstill.
    inline constexpr Real kSlipSpeedFloor = 1e-3;

    // A wheel rolling freely at forward speed v (m/s): spinning at v / R with no slip.
    inline wheel_state rolling_wheel(const wheel_params &w, Real v)
    {
        return {v / w.radius, 0, 0};
    }

    // Speed at which the tread passes through the contact patch, the larger of the road speed and the wheel's
    // surface speed. The slips are measured against it, and it sets how fast they relax.
    inline Real rolling_speed(const wheel_params &w, Real omega, Real vx)
    {
        return std::max(std::fabs(vx), std::fabs(omega * w.radius));
    }

    // Steady slip ratio (omega R - vx) / max(|vx|, |omega R|, floor): 0 rolling freely, -1 a locked wheel
    // sliding forward, +1 a wheel spinning at standstill. The book's form, which cannot divide by zero.
    inline Real steady_slip_ratio(const wheel_params &w, Real omega, Real vx)
    {
        return (omega * w.radius - vx) / std::max(rolling_speed(w, omega, vx), kSlipSpeedFloor);
    }

    // Steady lateral slip -vy / max(|vx|, |omega R|, floor): the tangent of the slip angle. Positive when the
    // patch moves to the right of where the wheel points, so the wheel points left of its path.
    inline Real steady_lateral_slip(const wheel_params &w, Real omega, const patch_velocity &v)
    {
        return -v.vy / std::max(rolling_speed(w, omega, v.vx), kSlipSpeedFloor);
    }

    // Relaxation of the slips towards their steady values over the relaxation lengths:
    //   sigma_x d(sx)/dt + V sx = omega R - vx        sigma_y d(sy)/dt + V sy = -vy        (V = rolling speed)
    // so the slip builds over a rolling distance, not a time. At standstill (V = 0) they integrate the patch
    // creep instead of dividing by zero: the tyre acts as a spring. Backward Euler, stable for any dt and V.
    inline void relax_slips(wheel_state &s, const wheel_params &w, const tyre_params &t, const patch_velocity &v, Real dt)
    {
        const Real V = rolling_speed(w, s.omega, v.vx);
        s.slip_ratio = (s.slip_ratio + dt / t.relaxation_length_x * (s.omega * w.radius - v.vx)) /
                       (1 + dt * V / t.relaxation_length_x);
        s.lateral_slip = (s.lateral_slip + dt / t.relaxation_length_y * (-v.vy)) /
                         (1 + dt * V / t.relaxation_length_y);
    }

    // Spin update: I domega/dt = T_drive - T_brake - Fx R. The brake torque (a magnitude) opposes the spin. If
    // it is enough to stop the wheel within this step it stops it exactly and holds it, so the brake can
    // stop the wheel but never reverse it.
    inline void spin_wheel(wheel_state &s, const wheel_params &w, Real fx, Real drive_torque, Real brake_torque, Real dt)
    {
        assert(brake_torque >= 0);
        const Real free = s.omega + dt * (drive_torque - fx * w.radius) / w.inertia;
        const Real brake = dt * brake_torque / w.inertia;
        s.omega = (std::fabs(free) <= brake) ? static_cast<Real>(0) : free - std::copysign(brake, free);
    }

    // One step of the wheel. The tyre force comes from the slips at the start of the step (that force goes to
    // the car); the spin is updated with it, then the slips relax with the new spin. The same velocity-first
    // order as the rigid body.
    inline tyre_force step_wheel(wheel_state &s, const wheel_params &w, const tyre_params &t, const patch_velocity &v,
                                 Real fz, Real drive_torque, Real brake_torque, Real dt)
    {
        const tyre_force f = tyre_forces(t, s.slip_ratio, s.lateral_slip, fz);
        spin_wheel(s, w, f.fx, drive_torque, brake_torque, dt);
        relax_slips(s, w, t, v, dt);
        return f;
    }
}
