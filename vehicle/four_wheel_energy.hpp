#pragma once
#include <cmath>
#include "vehicle/four_wheel.hpp"

namespace rallysim
{
    // The four-wheel car's energy books (energy accounting). The energy stored in the car can only
    // change by the work done on it: the drive puts energy in; the brakes, the dampers and the tyres' sliding take it
    // out; gravity and the springs only move it around. So over a run
    //     residual = (E(t) - E(0)) - integral of (drive - brake - damper - tyre) dt
    // stays near zero. energy_of() is E, rates_of() the integrand. A residual that drifts is a physics bug: a sign
    // error, a force applied at the wrong point, an integrator leak. "The quickest single detector of physics
    // bugs", and phase 7's gate is this balance.
    //
    // Expect small departures, not a drift: step() applies each tyre's force averaged over the wheel's sub-steps
    // while rates_of() uses the force at the start of the step (first-order error, shrinking with dt), and the
    // relaxed slips (the tyre's twist) store a little energy, so the tyre can briefly give some back, as when it
    // unwinds after a stop.

    // The energy stored in the four-wheel car at one instant (J), term by term (energy accounting).
    struct stored_energy
    {
        Real translation = 0; // body: 1/2 m |v|^2
        Real rotation = 0;    // body: 1/2 (Ix wx^2 + Iy wy^2 + Iz wz^2), omega in the body frame
        Real gravity = 0;     // body: m g z (springs model only; the planar model cancels gravity)
        Real springs = 0;     // corners: 1/2 k x^2 for each compressed spring (springs model only)
        Real wheels = 0;      // each wheel's spin: 1/2 I omega^2

        Real total() const { return translation + rotation + gravity + springs + wheels; }
    };

    inline stored_energy energy_of(const four_wheel_state &s, const four_wheel_params &p)
    {
        stored_energy e;
        const Real m = p.chassis.mass;
        const vec3 &I = p.chassis.inertia; // principal moments about body x, y, z
        const vec3 &w = s.body.omega;      // body frame, the same axes as I

        e.translation = 0.5 * m * dot(s.body.vel, s.body.vel);
        e.rotation = 0.5 * (I.x * w.x * w.x + I.y * w.y * w.y + I.z * w.z * w.z);

        if (p.vertical == vertical_model::springs)
        {
            e.gravity = m * kGravity * s.body.pos.z; // zero height at the ground
            for (std::size_t i = 0; i < kWheels; ++i)
            {
                const Real x = contact_of(p, s.body, i, 0.0).deflection; // compression; the steer does not change it
                if (x > 0)                                               // a lifted corner stores nothing
                    e.springs += 0.5 * p.corner(i).spring_rate * x * x;
            }
        }

        if (p.vertical == vertical_model::unsprung)
        {
            // the body moves vertically with the sprung mass only, each wheel with its own
            const Real ms = p.sprung_mass();
            e.translation = 0.5 * m * (s.body.vel.x * s.body.vel.x + s.body.vel.y * s.body.vel.y) +
                            0.5 * ms * s.body.vel.z * s.body.vel.z;
            vec3 cs{0, 0, 0}; // the sprung body's CG in the body frame: the whole car's less the wheels
            for (std::size_t i = 0; i < kWheels; ++i)
                cs = cs - p.corner(i).unsprung_mass * p.wheel_position(i);
            cs = (1.0 / ms) * cs;
            e.gravity = ms * kGravity * (s.body.pos.z + to_world(s.body.q, cs).z);
            for (std::size_t i = 0; i < kWheels; ++i)
            {
                const corner_params &c = p.corner(i);
                const corner_state &wc = s.corners[i];
                e.translation += 0.5 * c.unsprung_mass * wc.vz * wc.vz;
                e.gravity += c.unsprung_mass * kGravity * wc.z;

                const Real mount = p.static_deflection(i) - contact_of(p, s.body, i, 0.0).deflection;
                const Real x = p.strut_static_deflection(i) - mount + (wc.z - p.wheel_rest_height(i));
                const Real d = p.wheel.radius - wc.z;
                const Real stop = x - (p.strut_static_deflection(i) - c.droop); // the stop's travel, negative when engaged
                if (x > 0)
                    e.springs += 0.5 * c.spring_rate * x * x;
                if (d > 0)
                    e.springs += 0.5 * c.tyre_rate * d * d;
                if (stop < 0)
                    e.springs += 0.5 * c.stop_rate * stop * stop;
            }
        }

        for (const wheel_state &wheel : s.wheels)
            e.wheels += 0.5 * p.wheel.inertia * wheel.omega * wheel.omega;

        return e;
    }

    // The rates (W) at which energy enters and leaves the car at one instant, from the forces step() applies
    // (evaluate()). Each is written so a normal value is positive: drive puts energy in, the rest take it out.
    // Gravity and the springs are not here: their energy is stored (stored_energy), not lost.
    struct energy_rates
    {
        Real drive = 0;  // in: drive torque times each wheel's spin
        Real brake = 0;  // out: brake torque times each wheel's spin (the brakes always oppose it)
        Real damper = 0; // out: what the corners push with beyond their springs, times the compression speed
        Real tyre = 0;   // out: tyre force times slip velocity, the energy burnt in the contact patches

        Real net() const { return drive - brake - damper - tyre; } // the rate the stored energy should change at
    };

    inline energy_rates rates_of(const four_wheel_state &s, const four_wheel_params &p, const four_wheel_forces &f)
    {
        energy_rates r;
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            const Real omega = s.wheels[i].omega;
            r.drive += f.torque[i].drive * omega;
            r.brake += f.torque[i].brake * std::fabs(omega);

            // the corner pushes the body with fz; its spring part is stored energy, the rest is the damper
            if (p.vertical == vertical_model::springs && f.corner[i].deflection > 0)
                r.damper += (f.fz[i] - f.corner[i].spring) * f.corner[i].deflection_rate;

            // unsprung: the strut's damper, and the tyre's damping (its load less its spring) while it is on the ground
            if (p.vertical == vertical_model::unsprung)
            {
                const corner_load &l = f.corner[i];
                r.damper += l.damper * l.deflection_rate;
                if (l.tyre_deflection > 0)
                    r.damper += (f.fz[i] - p.corner(i).tyre_rate * l.tyre_deflection) * l.tyre_deflection_rate;
            }

            // the tyre pushes the body at the patch (fx vx + fy vy) and the wheel back (- fx R omega);
            // together they lose fx (omega R - vx) - fy vy, the force times how fast the patch slides
            const Real slip_x = omega * p.wheel.radius - f.patch[i].vx; // + when the wheel runs faster than the road
            r.tyre += f.tyre[i].fx * slip_x - f.tyre[i].fy * f.patch[i].vy;
        }
        return r;
    }

    // The work done on the car since the start of a run (J), by kind: the integrals of energy_rates.
    struct energy_work
    {
        Real drive = 0, brake = 0, damper = 0, tyre = 0;

        Real net() const { return drive - brake - damper - tyre; } // what the stored energy should have changed by
    };

    // The books of one run: the energy at the start and the work done since. Each step, before step(), add() the
    // rates of that step's forces (rates_of(s, p, evaluate(s, p, in))) times dt; residual() then says how far the
    // stored energy is from E(0) plus that work. Rectangle rule with the rates at the start of each step, like the
    // semi-implicit Euler step itself: first-order, so the residual shrinks with dt.
    class energy_books
    {
    public:
        energy_books(const four_wheel_state &start, const four_wheel_params &p) : start_(energy_of(start, p).total()) {}

        void add(const energy_rates &r, Real dt)
        {
            work_.drive += r.drive * dt;
            work_.brake += r.brake * dt;
            work_.damper += r.damper * dt;
            work_.tyre += r.tyre * dt;
        }

        Real start() const { return start_; }
        const energy_work &work() const { return work_; }

        // (E(now) - E(0)) - net work: zero if no energy is created or lost unaccounted (J)
        Real residual(const stored_energy &now) const { return now.total() - start_ - work_.net(); }

    private:
        Real start_;       // E(0), J
        energy_work work_; // since the start, J
    };
}