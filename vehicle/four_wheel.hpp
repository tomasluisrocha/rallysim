#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include "core/rigid_body.hpp"
#include "vehicle/tyre.hpp"
#include "vehicle/wheel.hpp"

namespace rallysim
{
    // Four-wheel car, planar (book section 7.1, phase 4 step 3): four contact patches, each with its own wheel
    // spin and tyre, on the rigid body with gravity cancelled so the car stays on the ground plane, as the
    // single-track does. Normal loads still come from the quasi-static load transfer; springs come in step 6.

    // Wheel order, the same everywhere: front-left, front-right, rear-left, rear-right.
    enum wheel_id : std::size_t
    {
        FL = 0,
        FR = 1,
        RL = 2,
        RR = 3
    };
    inline constexpr std::size_t kWheels = 4;

    inline bool is_front(std::size_t i) { return i == FL || i == FR; }
    inline bool is_left(std::size_t i) { return i == FL || i == RL; }

    struct four_wheel_params
    {
        rigid_body_params chassis{1200, {400, 1800, 1500}}; // mass (kg), inertia about x, y, z (kg m^2)
        Real lf = 1.3;                                      // CG to front axle (m)
        Real lr = 1.2;                                      // CG to rear axle (m)
        Real track_front = 1.5;                             // between the front contact patches (m)
        Real track_rear = 1.5;                              // between the rear contact patches (m)
        Real cg_height = 0.5;                               // CG above the ground (m)
        Real front_transfer_share = 0.5;                    // fraction of the lateral load transfer on the front axle
        Real max_steer = 0.6;                               // steer input clamped to +-max_steer (rad)
        wheel_params wheel;                                 // the same wheel at each corner
        tyre_params front_tyre{90000, 30000};               // half of the single-track front axle (60 kN/rad)
        tyre_params rear_tyre{90000, 35000};                // half of the single-track rear axle (70 kN/rad)

        Real wheelbase() const { return lf + lr; }
        const tyre_params &tyre(std::size_t i) const { return is_front(i) ? front_tyre : rear_tyre; }

        // Where the contact patch is, from the CG, in the body frame (m). At CG height for now, like the
        // single-track, so the tyre forces cannot roll or pitch the flat body; step 6 moves it to the ground.
        vec3 wheel_position(std::size_t i) const
        {
            const Real x = is_front(i) ? lf : -lr;
            const Real half_track = 0.5 * (is_front(i) ? track_front : track_rear);
            return {x, is_left(i) ? half_track : -half_track, 0};
        }
    };

    struct four_wheel_state
    {
        rigid_body_state body;
        std::array<wheel_state, kWheels> wheels{};
    };

    struct four_wheel_input
    {
        Real steer = 0;        // front wheel angle, rad (+left); both front wheels turn by it (no Ackermann yet)
        Real drive_torque = 0; // total torque on the rear wheels (N m, +forward), shared equally; step 4 adds a split
    };

    // Blend of two inputs for keyframe scripts: a at f = 0, b at f = 1.
    inline four_wheel_input lerp(const four_wheel_input &a, const four_wheel_input &b, Real f)
    {
        const Real g = static_cast<Real>(1) - f;
        return {g * a.steer + f * b.steer, g * a.drive_torque + f * b.drive_torque};
    }

    // A car on the ground plane, pointing along +x, moving forward at `speed` with every wheel rolling freely.
    inline four_wheel_state four_wheel_initial(const four_wheel_params &p, Real speed)
    {
        four_wheel_state s;
        s.body.vel = {speed, 0, 0};
        for (wheel_state &w : s.wheels)
            w = rolling_wheel(p.wheel, speed);
        return s;
    }

    inline Real applied_steer(const four_wheel_params &p, Real steer)
    {
        return std::clamp(steer, -p.max_steer, p.max_steer);
    }

    inline Real steer_angle(std::size_t i, Real steer)
    {
        return is_front(i) ? steer : static_cast<Real>(0);
    }

    // Torque on each wheel from the input: the drive torque shared between the two rear wheels.
    inline Real drive_torque(std::size_t i, const four_wheel_input &in)
    {
        return is_front(i) ? static_cast<Real>(0) : static_cast<Real>(0.5) * in.drive_torque;
    }

    // Velocity of a wheel's contact patch over the ground, in that wheel's frame (m/s): v_body + omega x r,
    // turned by the wheel's steer angle. In a left turn the right wheels move faster than the left ones.
    inline patch_velocity patch_velocity_of(const four_wheel_params &p, const rigid_body_state &body, std::size_t i, Real steer)
    {
        const vec3 v = to_body(body.q, body.vel) + cross(body.omega, p.wheel_position(i));
        const Real a = steer_angle(i, steer);
        const Real c = std::cos(a), s = std::sin(a);
        return {c * v.x + s * v.y, -s * v.x + c * v.y};
    }

    // Longitudinal acceleration used for the load transfer: the drive force the rear wheels can pass to the
    // road over the mass, with the rear-drive traction limit mu W lf / (L - mu h) at the rear tyre's nominal
    // friction (as the single-track). Lateral acceleration: forward speed times yaw rate.
    inline Real longitudinal_acceleration(const four_wheel_params &p, const four_wheel_input &in)
    {
        const Real mu = p.rear_tyre.friction_coefficient, h = p.cg_height, L = p.wheelbase();
        const Real weight = p.chassis.mass * kGravity;
        const Real all = mu * weight;
        const Real forward = (L - mu * h > 0) ? std::min(mu * weight * p.lf / (L - mu * h), all) : all;
        const Real backward = mu * weight * p.lf / (L + mu * h);
        return std::clamp(in.drive_torque / p.wheel.radius, -backward, forward) / p.chassis.mass;
    }

    // Vertical load on each wheel (N), quasi-static, the same model as the single-track: static axle loads,
    // then m a_x h / L from front to rear, then the roll moment m a_y h shared between the axles by
    // front_transfer_share and divided by each axle's track. No wheel and no axle goes below zero, and the
    // four always add up to the weight.
    inline std::array<Real, kWheels> four_wheel_loads(const four_wheel_params &p, Real a_x, Real a_y)
    {
        const Real weight = p.chassis.mass * kGravity, L = p.wheelbase(), h = p.cg_height;
        const Real front_static = weight * p.lr / L, rear_static = weight * p.lf / L;
        const Real shift = std::clamp(p.chassis.mass * a_x * h / L, -rear_static, front_static);
        const Real front = front_static - shift, rear = rear_static + shift;

        const Real roll_moment = p.chassis.mass * a_y * h;
        auto split = [](Real axle, Real transfer, Real &left, Real &right)
        {
            const Real half = std::max(axle, static_cast<Real>(0)) / 2;
            const Real moved = std::clamp(transfer, -half, half);
            left = half - moved;
            right = half + moved;
        };
        std::array<Real, kWheels> fz{};
        split(front, p.front_transfer_share * roll_moment / p.track_front, fz[FL], fz[FR]);
        split(rear, (1 - p.front_transfer_share) * roll_moment / p.track_rear, fz[RL], fz[RR]);
        return fz;
    }

    // Everything the car does in one step, computed once and shared by the step and the telemetry.
    struct four_wheel_forces
    {
        Real steer = 0;                            // front wheel angle used (rad)
        body_velocity vel;                         // velocity of the CG in the car's frame, and the yaw rate
        Real a_x = 0, a_y = 0;                     // accelerations used for the load transfer (m/s^2)
        std::array<Real, kWheels> fz{};            // vertical load on each wheel (N)
        std::array<patch_velocity, kWheels> patch; // contact patch velocity in each wheel's frame (m/s)
        std::array<tyre_force, kWheels> tyre;      // tyre force in each wheel's frame (N)
        std::array<Real, kWheels> torque{};        // drive torque on each wheel (N m)
    };

    // Order (book chapter 3): controls, normal loads, contact patch velocities, tyre forces from the wheels'
    // current slips. Nothing is changed; step() then applies it.
    inline four_wheel_forces evaluate(const four_wheel_state &s, const four_wheel_params &p, const four_wheel_input &in)
    {
        four_wheel_forces f;
        f.steer = applied_steer(p, in.steer);
        f.vel = velocity_in_body(s.body);
        f.a_x = longitudinal_acceleration(p, in);
        f.a_y = f.vel.u * f.vel.r;
        f.fz = four_wheel_loads(p, f.a_x, f.a_y);
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            f.patch[i] = patch_velocity_of(p, s.body, i, f.steer);
            f.tyre[i] = tyre_forces(p.tyre(i), s.wheels[i].slip_ratio, s.wheels[i].lateral_slip, f.fz[i]);
            f.torque[i] = drive_torque(i, in);
        }
        return f;
    }

    // Force and torque on the body from the four tyres: each force turned from its wheel's frame into the body
    // frame and applied at its contact patch, in the fixed wheel order. Gravity is cancelled.
    inline wrench four_wheel_wrench(const four_wheel_state &s, const four_wheel_params &p, const four_wheel_forces &f)
    {
        wrench w{};
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            const Real a = steer_angle(i, f.steer);
            const Real c = std::cos(a), sn = std::sin(a);
            const vec3 body_force{c * f.tyre[i].fx - sn * f.tyre[i].fy, sn * f.tyre[i].fx + c * f.tyre[i].fy, 0};
            add_force_at(w, s.body.q, to_world(s.body.q, body_force), p.wheel_position(i));
        }
        w.force.z += p.chassis.mass * kGravity; // the car stays on the ground plane for now
        return w;
    }

    // One step: the body moves with the forces of the step's start, each wheel's spin is updated with its own
    // tyre force (-Fx R) and torque, then its slips relax with the patch velocity of the step's start.
    inline void step(four_wheel_state &s, const four_wheel_params &p, const four_wheel_input &in, Real dt)
    {
        const four_wheel_forces f = evaluate(s, p, in);
        const wrench w = four_wheel_wrench(s, p, f);
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            spin_wheel(s.wheels[i], p.wheel, f.tyre[i].fx, f.torque[i], 0, dt);
            relax_slips(s.wheels[i], p.wheel, p.tyre(i), f.patch[i], dt);
        }
        step(s.body, p.chassis, w, dt);
    }
}
