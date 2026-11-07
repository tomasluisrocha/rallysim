#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include "core/rigid_body.hpp"
#include "vehicle/drive_brake.hpp"
#include "vehicle/tyre.hpp"
#include "vehicle/wheel.hpp"

namespace rallysim
{
    // Four-wheel car, planar (book section 7.1, phase 4 steps 3 and 4): four contact patches, each with its own
    // wheel spin and tyre, on the rigid body with gravity cancelled so the car stays on the ground plane, as
    // the single-track does. Driven by pedals through vehicle/drive_brake.hpp. Normal loads still come from the
    // quasi-static load transfer; springs come in step 6.

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
        drive_brake_params drive_brake;                     // pedals to wheel torques; rear drive by default

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

    // What the driver does: the steering wheel and three pedals (book section 11.1). The clutch comes with the
    // gearbox in phase 7.
    struct four_wheel_input
    {
        Real steer = 0;     // front wheel angle, rad (+left); both front wheels turn by it (no Ackermann yet)
        Real throttle = 0;  // 0 to 1
        Real brake = 0;     // 0 to 1
        Real handbrake = 0; // 0 to 1, rear wheels only
    };

    inline pedals pedals_of(const four_wheel_input &in)
    {
        return {in.throttle, in.brake, in.handbrake};
    }

    // Blend of two inputs for keyframe scripts: a at f = 0, b at f = 1.
    inline four_wheel_input lerp(const four_wheel_input &a, const four_wheel_input &b, Real f)
    {
        const Real g = static_cast<Real>(1) - f;
        return {g * a.steer + f * b.steer, g * a.throttle + f * b.throttle,
                g * a.brake + f * b.brake, g * a.handbrake + f * b.handbrake};
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

    // Velocity of a wheel's contact patch over the ground, in that wheel's frame (m/s): v_body + omega x r,
    // turned by the wheel's steer angle. In a left turn the right wheels move faster than the left ones.
    inline patch_velocity patch_velocity_of(const four_wheel_params &p, const rigid_body_state &body, std::size_t i, Real steer)
    {
        const vec3 v = to_body(body.q, body.vel) + cross(body.omega, p.wheel_position(i));
        const Real a = steer_angle(i, steer);
        const Real c = std::cos(a), s = std::sin(a);
        return {c * v.x + s * v.y, -s * v.x + c * v.y};
    }

    // Static axle loads moved by a longitudinal acceleration a_x: m a_x h / L from front to rear, never below zero.
    inline void axle_loads(const four_wheel_params &p, Real a_x, Real &front, Real &rear)
    {
        const Real weight = p.chassis.mass * kGravity, L = p.wheelbase();
        const Real front_static = weight * p.lr / L, rear_static = weight * p.lf / L;
        const Real shift = std::clamp(p.chassis.mass * a_x * p.cg_height / L, -rear_static, front_static);
        front = front_static - shift;
        rear = rear_static + shift;
    }

    // Longitudinal acceleration used for the load transfer, estimated from the wheel torques before the tyre
    // forces are known. Each axle asks for (drive - brake) / R, the brakes pushing against the direction of
    // travel; each axle can pass at most mu times its load, and that load depends on the acceleration. A few
    // fixed-point passes settle it (each pass shrinks the error by mu h / L, about 0.2), which gives the
    // traction limit of any drive split or brake bias: mu W lf / (L - mu h) for rear drive, mu W lr / (L + mu h)
    // for front drive, mu W for four-wheel drive. Uses the tyres' nominal friction (no load sensitivity).
    // Lateral acceleration is estimated separately, as forward speed times yaw rate.
    //
    // The brakes' direction fades out below kBrakeFadeSpeed: brakes holding a car at rest push nothing, and the
    // estimate must not jump from front to rear the moment the car stops.
    inline constexpr Real kBrakeFadeSpeed = 0.5; // m/s

    inline Real longitudinal_acceleration(const four_wheel_params &p, const std::array<wheel_torque, kWheels> &t, Real u)
    {
        const Real against = -std::clamp(u / kBrakeFadeSpeed, static_cast<Real>(-1), static_cast<Real>(1));
        const Real R = p.wheel.radius;
        const Real front_ask = (t[FL].drive + t[FR].drive + against * (t[FL].brake + t[FR].brake)) / R;
        const Real rear_ask = (t[RL].drive + t[RR].drive + against * (t[RL].brake + t[RR].brake)) / R;

        Real a_x = 0;
        for (int pass = 0; pass < 6; ++pass)
        {
            Real front, rear;
            axle_loads(p, a_x, front, rear);
            const Real front_max = p.front_tyre.friction_coefficient * front;
            const Real rear_max = p.rear_tyre.friction_coefficient * rear;
            a_x = (std::clamp(front_ask, -front_max, front_max) + std::clamp(rear_ask, -rear_max, rear_max)) / p.chassis.mass;
        }
        return a_x;
    }

    // Vertical load on each wheel (N), quasi-static, the same model as the single-track: static axle loads,
    // then m a_x h / L from front to rear, then the roll moment m a_y h shared between the axles by
    // front_transfer_share and divided by each axle's track. No wheel and no axle goes below zero, and the
    // four always add up to the weight.
    inline std::array<Real, kWheels> four_wheel_loads(const four_wheel_params &p, Real a_x, Real a_y)
    {
        const Real h = p.cg_height;
        Real front, rear;
        axle_loads(p, a_x, front, rear);

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
        std::array<wheel_torque, kWheels> torque;  // drive and brake torque on each wheel (N m)
    };

    // Order (book chapter 3): controls, normal loads, contact patch velocities, tyre forces from the wheels'
    // current slips. Nothing is changed; step() then applies it.
    inline four_wheel_forces evaluate(const four_wheel_state &s, const four_wheel_params &p, const four_wheel_input &in)
    {
        four_wheel_forces f;
        f.steer = applied_steer(p, in.steer);
        f.vel = velocity_in_body(s.body);
        f.torque = wheel_torques(p.drive_brake, pedals_of(in),
                                 {s.wheels[FL].omega, s.wheels[FR].omega, s.wheels[RL].omega, s.wheels[RR].omega});
        f.a_x = longitudinal_acceleration(p, f.torque, f.vel.u);
        f.a_y = f.vel.u * f.vel.r;
        f.fz = four_wheel_loads(p, f.a_x, f.a_y);
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            f.patch[i] = patch_velocity_of(p, s.body, i, f.steer);
            f.tyre[i] = tyre_forces(p.tyre(i), s.wheels[i].slip_ratio, s.wheels[i].lateral_slip, f.fz[i]);
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
    // tyre force (-Fx R), drive torque and brake torque (the brake stops a wheel but never reverses it), then
    // its slips relax with the patch velocity of the step's start.
    inline void step(four_wheel_state &s, const four_wheel_params &p, const four_wheel_input &in, Real dt)
    {
        const four_wheel_forces f = evaluate(s, p, in);
        const wrench w = four_wheel_wrench(s, p, f);
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            spin_wheel(s.wheels[i], p.wheel, f.tyre[i].fx, f.torque[i].drive, f.torque[i].brake, dt);
            relax_slips(s.wheels[i], p.wheel, p.tyre(i), f.patch[i], dt);
        }
        step(s.body, p.chassis, w, dt);
    }
}
