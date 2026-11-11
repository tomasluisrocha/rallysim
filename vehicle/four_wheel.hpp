#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include "core/keyframe_script.hpp"
#include "core/rigid_body.hpp"
#include "vehicle/corner.hpp"
#include "vehicle/drive_brake.hpp"
#include "vehicle/load_transfer.hpp"
#include "vehicle/tyre.hpp"
#include "vehicle/wheel.hpp"

namespace rallysim
{
    // Four-wheel car (phase 4): four contact patches, each with its own wheel spin and tyre,
    // driven by pedals through vehicle/drive_brake.hpp. Two ways to get the vertical loads:
    //   springs (step 6, the default): gravity acts, each corner of the body sits on a spring and damper
    //     (vehicle/corner.hpp) and the tyre forces act on the ground, below the CG, so the body heaves, rolls and
    //     pitches and the load transfer comes out of the springs;
    //   planar (steps 3 to 5): gravity is cancelled and the tyre forces act at CG height, so the body stays flat
    //     on the ground plane, as the single-track; the loads come from the quasi-static load transfer.
    //   unsprung (phase 6): as springs, but each wheel has its own mass on a tyre spring below the strut, so the
    //     tyre load is the tyre spring's and the wheel can hop and hang (see "the corner with a wheel" below).
    // The planar model stays so the four-wheel car can still be compared one to one with the single-track, and
    // the springs model so the unsprung one can be compared with it.
    enum class vertical_model
    {
        planar,
        springs,
        unsprung
    };

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

    // The car's tyre (phase 5): the tmeasy curve, with its own peak and fall-off along and across the wheel, given
    // at two loads. Its tables are tyre_params' defaults with the stiffnesses scaled so that at `static_load`, the
    // load the wheel carries with the car at rest, they are the given ones; the defaults' growth with load is kept.
    // A tmeasy tyre's stiffness depends on its load, the linear and peak_slide tyres' does not. Scaled at the
    // nominal load instead (3000 N), the front tyres (2825 N at rest) came out 5% softer and the rears (3061 N) 2%
    // stiffer than the same car on the linear tyre, which doubled the understeer gradient. Scaled at the static
    // load, the car at rest has exactly the linear car's stiffnesses, so set_tyre_curve() still puts the same car on
    // the older curves, and what differs at speed is the tmeasy tyre itself (load transfer, the curve's bend).
    // The rest (grip, load sensitivity, relaxation) is the defaults.
    //
    // fit_stiffness does the scaling on a tyre in place: its tmeasy tables are scaled so that at `static_load` their
    // stiffnesses are the tyre's longitudinal_stiffness and cornering_stiffness. Only the stiffnesses change, so
    // grip and the rest stay as they are, and a tyre that already fits is left as it was.
    inline void fit_stiffness(tyre_params &t, Real static_load)
    {
        const Real along = t.longitudinal_stiffness / tmeasy_at(t.tmeasy_x, static_load, t.nominal_load).stiffness;
        const Real across = t.cornering_stiffness / tmeasy_at(t.tmeasy_y, static_load, t.nominal_load).stiffness;
        t.tmeasy_x.nominal.stiffness *= along;
        t.tmeasy_x.double_load.stiffness *= along;
        t.tmeasy_y.nominal.stiffness *= across;
        t.tmeasy_y.double_load.stiffness *= across;
    }

    inline tyre_params car_tyre(Real longitudinal_stiffness, Real cornering_stiffness, Real static_load)
    {
        tyre_params t;
        t.longitudinal_stiffness = longitudinal_stiffness;
        t.cornering_stiffness = cornering_stiffness;
        fit_stiffness(t, static_load);
        t.curve = tyre_curve::tmeasy;
        return t;
    }

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
        // half of the single-track front and rear axles (60 / 70 kN/rad), at each wheel's load at rest
        tyre_params front_tyre = car_tyre(90000, 30000, 0.5 * chassis.mass * kGravity * lr / (lf + lr));
        tyre_params rear_tyre = car_tyre(90000, 35000, 0.5 * chassis.mass * kGravity * lf / (lf + lr));
        drive_brake_params drive_brake;                    // pedals to wheel torques; rear drive by default
        vertical_model vertical = vertical_model::springs; // how the vertical loads are found (see above)
        corner_params front_corner;                        // suspension of each front corner (springs, unsprung)
        corner_params rear_corner;                         // and of each rear corner

        Real wheelbase() const { return lf + lr; }
        const tyre_params &tyre(std::size_t i) const { return is_front(i) ? front_tyre : rear_tyre; }
        // Both axles on one curve; the tests whose closed forms assume a flat ceiling put the car back on linear.
        void set_tyre_curve(tyre_curve c) { front_tyre.curve = rear_tyre.curve = c; }
        const corner_params &corner(std::size_t i) const { return is_front(i) ? front_corner : rear_corner; }

        // Where the contact patch is, from the CG, in the body frame (m), in the planar model: at CG height, like
        // the single-track, so the tyre forces cannot roll or pitch the flat body.
        vec3 wheel_position(std::size_t i) const
        {
            const Real x = is_front(i) ? lf : -lr;
            const Real half_track = 0.5 * (is_front(i) ? track_front : track_rear);
            return {x, is_left(i) ? half_track : -half_track, 0};
        }

        // The point on the body where a corner's spring is mounted, from the CG, body frame (m): straight above
        // the contact patch, at ground level when the car is level with its CG at cg_height (springs, unsprung).
        vec3 suspension_point(std::size_t i) const
        {
            const vec3 w = wheel_position(i);
            return {w.x, w.y, -cg_height};
        }

        // Each corner's share of the weight when the car stands still (N), and how far that compresses its
        // spring (m). The springs are set so the car rests level with its CG at cg_height.
        Real static_load(std::size_t i) const
        {
            return 0.5 * chassis.mass * kGravity * (is_front(i) ? lr : lf) / wheelbase();
        }
        Real static_deflection(std::size_t i) const { return static_load(i) / corner(i).spring_rate; }

        // The corner with a wheel (unsprung model). `chassis.mass` is the whole car, wheels included, and lf, lr
        // and cg_height locate the whole car's CG; the body alone has the sprung mass. At rest the car is level
        // with its CG at cg_height, as with springs: each tyre carries static_load(i), its wheel centre sits at
        // wheel_rest_height(i), and the strut carries that load less the wheel's own weight.
        Real sprung_mass() const { return chassis.mass - 2 * front_corner.unsprung_mass - 2 * rear_corner.unsprung_mass; }
        Real wheel_rest_height(std::size_t i) const { return wheel.radius - static_load(i) / corner(i).tyre_rate; }
        Real strut_static_deflection(std::size_t i) const
        {
            return (static_load(i) - corner(i).unsprung_mass * kGravity) / corner(i).spring_rate;
        }
    };

    // Refits both tyres' stiffness tables to the car as it is now (fit_stiffness at each axle's load at rest). The
    // tyres are fitted when the params are built, so after changing mass, lf, lr or a tyre's stiffness, call this
    // once, after all the changes: then the order of the changes does not matter.
    inline void rebuild_tyres(four_wheel_params &p)
    {
        fit_stiffness(p.front_tyre, p.static_load(FL));
        fit_stiffness(p.rear_tyre, p.static_load(RL));
    }

    // A wheel's vertical motion (unsprung model; zero and unused in the others).
    struct corner_state
    {
        Real z = 0;  // m, the wheel centre's height above the ground
        Real vz = 0; // m/s, + up
    };

    struct four_wheel_state
    {
        rigid_body_state body;
        std::array<wheel_state, kWheels> wheels{};
        std::array<corner_state, kWheels> corners{};
    };

    // What the driver does: the steering wheel and three pedals. The clutch comes with the
    // gearbox in phase 7.
    struct four_wheel_input
    {
        Real steer = 0;     // front wheel angle, rad (+left); both front wheels turn by it (no Ackermann yet)
        Real throttle = 0;  // 0 to 1
        Real brake = 0;     // 0 to 1
        Real handbrake = 0; // 0 to 1, rear wheels only
    };

    // The first field of the four-wheel state that is not a finite number, or nullptr if all are: the body, then
    // each wheel's spin, relaxed slips, height and vertical speed, in the fixed wheel order.
    inline const char *non_finite_field(const four_wheel_state &s)
    {
        if (const char *bad = non_finite_field(s.body))
            return bad;
        static const char *const names[kWheels][5] = {
            {"wheel FL omega", "wheel FL slip_ratio", "wheel FL lateral_slip", "wheel FL z", "wheel FL vz"},
            {"wheel FR omega", "wheel FR slip_ratio", "wheel FR lateral_slip", "wheel FR z", "wheel FR vz"},
            {"wheel RL omega", "wheel RL slip_ratio", "wheel RL lateral_slip", "wheel RL z", "wheel RL vz"},
            {"wheel RR omega", "wheel RR slip_ratio", "wheel RR lateral_slip", "wheel RR z", "wheel RR vz"}};
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            const wheel_state &w = s.wheels[i];
            if (!std::isfinite(w.omega))
                return names[i][0];
            if (!std::isfinite(w.slip_ratio))
                return names[i][1];
            if (!std::isfinite(w.lateral_slip))
                return names[i][2];
            if (!std::isfinite(s.corners[i].z))
                return names[i][3];
            if (!std::isfinite(s.corners[i].vz))
                return names[i][4];
        }
        return nullptr;
    }

    // Why the car is somewhere the model cannot represent, or nullptr if it is not. The springs and unsprung
    // models have no body contact: a car on its side or roof, or with its CG under the ground, would pass through
    // it. Planar never refuses (its body stays flat by construction).
    inline const char *outside_model(const four_wheel_state &s, const four_wheel_params &p)
    {
        if (p.vertical == vertical_model::planar)
            return nullptr;
        if (to_world(s.body.q, vec3{0, 0, 1}).z <= 0)
            return "the car is on its side or roof (no body contact yet)";
        if (s.body.pos.z <= 0)
            return "the CG is at or below the ground";
        return nullptr;
    }

    inline pedals pedals_of(const four_wheel_input &in)
    {
        return {in.throttle, in.brake, in.handbrake};
    }

    // Blend of two inputs for keyframe scripts: a at f = 0, b at f = 1.
    inline four_wheel_input lerp(const four_wheel_input &a, const four_wheel_input &b, Real f)
    {
        return {lerp(a.steer, b.steer, f), lerp(a.throttle, b.throttle, f), lerp(a.brake, b.brake, f),
                lerp(a.handbrake, b.handbrake, f)};
    }

    // Where a run starts: the CG's position and velocity (world frame), the body's attitude as yaw-pitch-roll angles
    // with ISO 8855 signs (as roll_angle and pitch_angle read them back), and its angular velocity (body frame).
    struct four_wheel_pose
    {
        vec3 position;          // m
        Real heading = 0;       // rad, counter-clockwise from +x
        Real pitch = 0;         // rad, + nose down
        Real roll = 0;          // rad, + right side down
        vec3 velocity;          // m/s
        vec3 angular_velocity;  // rad/s: roll, pitch and yaw rates (p, q, r)
    };

    // The state at `pose`, with every wheel rolling at the body's forward speed, so no tyre starts out slipping.
    // With every angle zero the orientation is exactly the identity, so a level start is bit for bit the same as an
    // unrotated one.
    inline four_wheel_state four_wheel_start(const four_wheel_params &p, const four_wheel_pose &pose)
    {
        four_wheel_state s;
        s.body.pos = pose.position;
        s.body.q = from_axis_angle({0, 0, 1}, pose.heading) * from_axis_angle({0, 1, 0}, pose.pitch) *
                   from_axis_angle({1, 0, 0}, pose.roll);
        s.body.vel = pose.velocity;
        s.body.omega = pose.angular_velocity;
        const Real forward = to_body(s.body.q, s.body.vel).x;
        for (wheel_state &w : s.wheels)
            w = rolling_wheel(p.wheel, forward);
        if (p.vertical == vertical_model::unsprung)
        {
            // each wheel at its rest position under its mount and moving with it: for a level car at cg_height
            // that is exactly rest
            const vec3 omega = to_world(s.body.q, s.body.omega);
            for (std::size_t i = 0; i < kWheels; ++i)
            {
                const vec3 mount = to_world(s.body.q, p.suspension_point(i));
                s.corners[i].z = s.body.pos.z + mount.z + p.wheel_rest_height(i);
                s.corners[i].vz = s.body.vel.z + cross(omega, mount).z;
            }
        }
        return s;
    }

    // A car pointing along +x, moving forward at `speed` with every wheel rolling freely. With springs (or the
    // unsprung model) it starts level with its CG at cg_height, each spring compressed by its share of the
    // weight: at rest, in balance.
    inline four_wheel_state four_wheel_initial(const four_wheel_params &p, Real speed)
    {
        four_wheel_pose pose;
        pose.position = {0, 0, p.vertical == vertical_model::planar ? 0 : p.cg_height};
        pose.velocity = {speed, 0, 0};
        return four_wheel_start(p, pose);
    }

    inline Real applied_steer(const four_wheel_params &p, Real steer)
    {
        return std::clamp(steer, -p.max_steer, p.max_steer);
    }

    inline Real steer_angle(std::size_t i, Real steer)
    {
        return is_front(i) ? steer : static_cast<Real>(0);
    }

    // Where a wheel touches the ground and how it moves there (steps 1 and 2).
    struct contact_geometry
    {
        vec3 point;               // the contact patch from the CG, body frame (m): where the tyre force acts
        vec3 forward, left;       // the wheel's directions along the ground, world frame, unit length
        patch_velocity velocity;  // how fast the patch moves over the ground, in the wheel's frame (m/s)
        Real deflection = 0;      // springs, unsprung: how far the mount's spring is compressed at rest geometry (m)
        Real deflection_rate = 0; // springs, unsprung: how fast it is being compressed (m/s)
    };

    // The contact of wheel i. The patch velocity is v_body + omega x r, with r from the CG to the patch, taken
    // along the wheel and across it: in a left turn the right wheels move faster than the left ones.
    //   planar: the patch is at CG height and the wheel's frame is the body's, turned by the steer.
    //   springs: the spring's mount (suspension_point) moves with the body; its height above the ground gives
    //   the compression and its vertical speed the compression rate. The patch is on the ground straight below
    //   it, and the wheel's direction is the body's heading turned by the steer, laid flat on the ground, so a
    //   rolled or pitched body does not tilt the tyre forces into the ground.
    inline contact_geometry contact_of(const four_wheel_params &p, const rigid_body_state &body, std::size_t i, Real steer)
    {
        contact_geometry c;
        const Real a = steer_angle(i, steer);
        const Real ca = std::cos(a), sa = std::sin(a);
        const vec3 omega = to_world(body.q, body.omega);
        vec3 r; // the patch from the CG, world frame
        if (p.vertical == vertical_model::planar)
        {
            c.point = p.wheel_position(i);
            r = to_world(body.q, c.point);
            c.forward = to_world(body.q, vec3{ca, sa, 0});
            c.left = to_world(body.q, vec3{-sa, ca, 0});
        }
        else
        {
            const vec3 mount = to_world(body.q, p.suspension_point(i));
            c.deflection = p.static_deflection(i) - (body.pos.z + mount.z);
            c.deflection_rate = -(body.vel.z + cross(omega, mount).z);
            r = vec3{mount.x, mount.y, -body.pos.z};
            c.point = to_body(body.q, r);
            const vec3 heading = to_world(body.q, vec3{ca, sa, 0});
            const Real n = std::hypot(heading.x, heading.y);
            c.forward = vec3{heading.x / n, heading.y / n, 0};
            c.left = vec3{-c.forward.y, c.forward.x, 0};
        }
        const vec3 v = body.vel + cross(omega, r);
        c.velocity = {dot(v, c.forward), dot(v, c.left)};
        return c;
    }

    inline patch_velocity patch_velocity_of(const four_wheel_params &p, const rigid_body_state &body, std::size_t i, Real steer)
    {
        return contact_of(p, body, i, steer).velocity;
    }

    // Static axle loads moved by a longitudinal acceleration a_x (vehicle/load_transfer.hpp).
    inline void axle_loads(const four_wheel_params &p, Real a_x, Real &front, Real &rear)
    {
        longitudinal_axle_loads(p.chassis.mass, p.lf, p.lr, p.cg_height, a_x, front, rear);
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
        std::array<Real, kWheels> fz{};
        split_axle(front, p.front_transfer_share * roll_moment / p.track_front, fz[FL], fz[FR]);
        split_axle(rear, (1 - p.front_transfer_share) * roll_moment / p.track_rear, fz[RL], fz[RR]);
        return fz;
    }

    // Everything the car does in one step, computed once and shared by the step and the telemetry.
    struct four_wheel_forces
    {
        Real steer = 0;                            // front wheel angle used (rad)
        body_velocity vel;                         // velocity of the CG in the car's frame, and the yaw rate
        Real a_x = 0, a_y = 0;                     // planar model: accelerations used for the load transfer (m/s^2)
        std::array<Real, kWheels> fz{};            // vertical load on each wheel (N)
        std::array<corner_load, kWheels> corner;   // springs, unsprung: each corner's spring, damper and (unsprung) stop and tyre
        std::array<patch_velocity, kWheels> patch; // contact patch velocity in each wheel's frame (m/s)
        std::array<tyre_force, kWheels> tyre;      // tyre force in each wheel's frame (N)
        std::array<wheel_torque, kWheels> torque;  // drive and brake torque on each wheel (N m)
    };

    // Order: controls, contact geometry, normal loads, contact patch velocities, tyre forces
    // from the wheels' current slips. Nothing is changed; step() then applies it.
    inline four_wheel_forces evaluate(const four_wheel_state &s, const four_wheel_params &p, const four_wheel_input &in)
    {
        four_wheel_forces f;
        f.steer = applied_steer(p, in.steer);
        f.vel = velocity_in_body(s.body);
        f.torque = wheel_torques(p.drive_brake, pedals_of(in),
                                 {s.wheels[FL].omega, s.wheels[FR].omega, s.wheels[RL].omega, s.wheels[RR].omega});
        f.a_x = longitudinal_acceleration(p, f.torque, f.vel.u);
        f.a_y = f.vel.u * f.vel.r;
        if (p.vertical == vertical_model::planar)
            f.fz = four_wheel_loads(p, f.a_x, f.a_y);
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            const contact_geometry c = contact_of(p, s.body, i, f.steer);
            if (p.vertical == vertical_model::springs)
            {
                f.corner[i] = corner_force(p.corner(i), c.deflection, c.deflection_rate);
                f.fz[i] = f.corner[i].fz;
            }
            else if (p.vertical == vertical_model::unsprung)
            {
                // the mount's height above its rest height, and its vertical speed, from the springs geometry
                const Real mount = p.static_deflection(i) - c.deflection, mount_rate = -c.deflection_rate;
                const corner_state &wheel = s.corners[i];
                const Real strut = p.strut_static_deflection(i) - mount + (wheel.z - p.wheel_rest_height(i));
                f.corner[i] = unsprung_corner_force(p.corner(i), strut, wheel.vz - mount_rate, p.strut_static_deflection(i),
                                                    p.wheel.radius - wheel.z, -wheel.vz);
                f.fz[i] = f.corner[i].fz; // the tyre spring's force
            }
            f.patch[i] = c.velocity;
            f.tyre[i] = wheel_force(s.wheels[i], p.wheel, p.tyre(i), f.patch[i], f.fz[i]);
        }
        return f;
    }

    // Force and torque on the body from the four wheels, in the fixed wheel order (step 5):
    // each tyre force along the ground, (Fx forward + Fy left), applied at its contact patch. With springs each
    // corner's load also pushes up at its patch, and gravity acts; in the planar model gravity is cancelled.
    //
    // Unsprung: the strut's force pushes up at the patch instead of the tyre load (that one acts on the wheel).
    // The body also gets each wheel's weight, upwards, at its corner at CG height. lf, lr and cg_height locate the
    // whole car's CG, not the sprung body's, and that term is what makes the strut forces balance about it: at
    // rest the two together are the tyre's static load, as with springs, and the car sits level.
    // Last, the vertical force is rescaled. The rigid-body step divides the force by the whole mass and subtracts
    // g; the body's vertical mass is the sprung mass, ms a_z = (sum of the strut forces) - ms g. With V the sum
    // of the strut forces and the wheels' weights, that is a_z = (V - m g) / ms, which the step gives from a
    // force of (m / ms) (V - m g) + m g. Along the ground the whole mass moves, as before.
    inline wrench four_wheel_wrench(const four_wheel_state &s, const four_wheel_params &p, const four_wheel_forces &f)
    {
        wrench w{};
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            const contact_geometry c = contact_of(p, s.body, i, f.steer);
            vec3 force = f.tyre[i].fx * c.forward + f.tyre[i].fy * c.left;
            if (p.vertical == vertical_model::springs)
                force.z += f.fz[i];
            else if (p.vertical == vertical_model::unsprung)
                force.z += f.corner[i].strut;
            add_force_at(w, s.body.q, force, c.point);
            if (p.vertical == vertical_model::unsprung)
                add_force_at(w, s.body.q, vec3{0, 0, p.corner(i).unsprung_mass * kGravity}, p.wheel_position(i));
        }
        if (p.vertical == vertical_model::planar)
            w.force.z += p.chassis.mass * kGravity; // the flat car stays on the ground plane
        else if (p.vertical == vertical_model::unsprung)
        {
            const Real weight = p.chassis.mass * kGravity;
            w.force.z = p.chassis.mass / p.sprung_mass() * (w.force.z - weight) + weight;
        }
        return w;
    }

    // One step. The loads, patch velocities and torques come from the step's start (evaluate). Each wheel then
    // steps on its own, in sub-steps (step_wheel): its spin changes with its tyre force (-Fx R), drive torque and
    // brake torque (the brake stops a wheel but never reverses it), and its slips relax. The body then moves with
    // each wheel's average tyre force over the step, so it gets the same impulse the wheels felt. Unsprung: each
    // wheel then moves vertically between its tyre and its strut, velocity first, with the same start-of-step forces.
    inline void step(four_wheel_state &s, const four_wheel_params &p, const four_wheel_input &in, Real dt)
    {
        four_wheel_forces f = evaluate(s, p, in);
        for (std::size_t i = 0; i < kWheels; ++i)
            f.tyre[i] = step_wheel(s.wheels[i], p.wheel, p.tyre(i), f.patch[i], f.fz[i], f.torque[i].drive,
                                   f.torque[i].brake, dt);
        const wrench w = four_wheel_wrench(s, p, f);
        step(s.body, p.chassis, w, dt);
        if (p.vertical == vertical_model::unsprung)
            for (std::size_t i = 0; i < kWheels; ++i)
            {
                corner_state &wheel = s.corners[i];
                wheel.vz += ((f.fz[i] - f.corner[i].strut) / p.corner(i).unsprung_mass - kGravity) * dt;
                wheel.z += wheel.vz * dt;
            }
    }
}
