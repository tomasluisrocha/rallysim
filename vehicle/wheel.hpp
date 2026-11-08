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
        int substeps = 4;    // spin updates per step: the wheel changes far faster than the body (book ch. 14)
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

    // The relaxed slips stand for how far the contact patch is twisted (sigma times the slip). A tyre cannot be
    // twisted further than its force needs: beyond that the patch only slides. So the slips are capped at the
    // normalised slip where the force stops changing (max_normalised_slip: 1 for the linear tyre, where it
    // reaches the grip; sliding_slip for peak_slide, where it slides fully). If (Cx sx, C_alpha sy) is longer
    // than that limit times the grip at load fz, both are scaled back along their direction. The force does not
    // change (the curve is flat there), but a tyre that stops sliding is left twisted by millimetres instead of
    // by sigma (20 cm for a locked wheel), which threw a stopped car backwards.
    inline void cap_slips(wheel_state &s, const tyre_params &t, Real fz)
    {
        const Real grip = tyre_grip(t, fz);
        const Real size = std::hypot(t.longitudinal_stiffness * s.slip_ratio, t.cornering_stiffness * s.lateral_slip);
        const Real limit = max_normalised_slip(t) * grip;
        if (size > limit)
        {
            const Real scale = limit / size; // zero in the air: an unloaded tyre is not twisted
            s.slip_ratio *= scale;
            s.lateral_slip *= scale;
        }
    }

    // Relaxation of the slips towards their steady values over the relaxation lengths:
    //   sigma_x d(sx)/dt + V sx = omega R - vx        sigma_y d(sy)/dt + V sy = -vy        (V = rolling speed)
    // so the slip builds over a rolling distance, not a time. At standstill (V = 0) they integrate the patch
    // creep instead of dividing by zero: the tyre acts as a spring. Backward Euler, stable for any dt and V;
    // then capped at what the grip at load fz can hold.
    inline void relax_slips(wheel_state &s, const wheel_params &w, const tyre_params &t, const patch_velocity &v, Real fz, Real dt)
    {
        const Real V = rolling_speed(w, s.omega, v.vx);
        s.slip_ratio = (s.slip_ratio + dt / t.relaxation_length_x * (s.omega * w.radius - v.vx)) /
                       (1 + dt * V / t.relaxation_length_x);
        s.lateral_slip = (s.lateral_slip + dt / t.relaxation_length_y * (-v.vy)) /
                         (1 + dt * V / t.relaxation_length_y);
        cap_slips(s, t, fz);
    }

    // The force of the tyre on the car (N, wheel frame): the twisted contact patch (tyre_forces), plus at low
    // speed a damping force against the patch's sliding, (omega R - vx) along the wheel and -vy across it. The
    // damping fades in linearly below damping_fade_speed, so at speed nothing changes; at standstill it soaks
    // up the patch's spring, so a stopped car settles instead of rocking. The total stays within the grip.
    //
    // Keep damping_x * R^2 * (dt / substeps) / I below about 0.5: the damping also acts on the wheel's spin
    // (through -Fx R), and above that the explicit spin update overshoots and blows up (book chapter 14).
    inline tyre_force wheel_force(const wheel_state &s, const wheel_params &w, const tyre_params &t,
                                  const patch_velocity &v, Real fz)
    {
        tyre_force f = tyre_forces(t, s.slip_ratio, s.lateral_slip, fz);
        const Real V = rolling_speed(w, s.omega, v.vx);
        const Real weight = std::clamp(1 - V / t.damping_fade_speed, static_cast<Real>(0), static_cast<Real>(1));
        if (weight > 0)
        {
            f.fx += weight * t.damping_x * (s.omega * w.radius - v.vx);
            f.fy += weight * t.damping_y * (-v.vy);
            const Real grip = tyre_grip(t, fz), size = std::hypot(f.fx, f.fy);
            if (size > grip)
            {
                f.fx *= grip / size;
                f.fy *= grip / size;
            }
        }
        return f;
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

    // One step of the wheel, in `substeps` smaller steps (book chapter 14: the wheel is a light flywheel with a
    // large force at its rim, so its spin changes much faster than the body's motion). Each sub-step takes the tyre
    // force from the wheel's current slips, updates the spin with it, then relaxes the slips with the new spin, the
    // same velocity-first order as the rigid body. The patch velocity, the load and the torques stay as given for
    // the whole step: the body barely moves in it. Returns the average tyre force over the step, so the impulse the
    // body gets is the one the wheel felt.
    inline tyre_force step_wheel(wheel_state &s, const wheel_params &w, const tyre_params &t, const patch_velocity &v,
                                 Real fz, Real drive_torque, Real brake_torque, Real dt)
    {
        assert(w.substeps >= 1);
        const Real h = dt / w.substeps;
        tyre_force sum;
        for (int k = 0; k < w.substeps; ++k)
        {
            const tyre_force f = wheel_force(s, w, t, v, fz);
            spin_wheel(s, w, f.fx, drive_torque, brake_torque, h);
            relax_slips(s, w, t, v, fz, h);
            sum.fx += f.fx;
            sum.fy += f.fy;
        }
        return {sum.fx / w.substeps, sum.fy / w.substeps};
    }
}
