#include <algorithm>
#include <cmath>
#include <cstdio>
#include "scenarios/single_track_lane_change.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// A car on the ground plane at heading `yaw`, with velocity (u, v) in its own frame and yaw rate r.
static rigid_body_state car(Real yaw, Real u, Real v, Real r)
{
    rigid_body_state s;
    s.q = from_axis_angle(vec3{0, 0, 1}, yaw);
    s.vel = to_world(s.q, vec3{u, v, 0});
    s.omega = vec3{0, 0, r};
    return s;
}

// Steady-state understeer gradient of the car, rad per m/s^2 of lateral acceleration.
static Real understeer_gradient(const single_track_params &p)
{
    const axle_pair w = axle_loads(p);
    return (w.front / p.front_cornering_stiffness - w.rear / p.rear_cornering_stiffness) / kGravity;
}

// Holds `speed` with a constant steer from the start and returns the state after `steps` steps.
static rigid_body_state steady_turn(const single_track_params &p, Real speed, Real steer, long steps)
{
    single_track_run r;
    r.params = p;
    r.initial = single_track_initial(speed);
    r.hold_speed = speed;
    r.script.keys = {{0.0, single_track_input{steer, 0.0}}};
    r.steps = steps;
    return run(r);
}

// Independent planar model: the book's three equations in the body frame, integrated on their own
// (explicit Euler, mid-step heading for the position). It shares only the tyre functions, which are
// tested above, with the rigid-body version.
struct planar_state
{
    Real x = 0, y = 0, yaw = 0, u = 0, v = 0, r = 0;
};

static void planar_step(planar_state &s, const single_track_params &p, const single_track_input &in, Real dt)
{
    const Real steer = applied_steer(p, in.steer);
    const axle_pair fy = lateral_forces(p, slip_angles(p, body_velocity{s.u, s.v, s.r}, steer), axle_loads(p));
    const Real fx = in.drive_force - fy.front * std::sin(steer);
    const Real fy_total = fy.front * std::cos(steer) + fy.rear;
    const Real mz = p.lf * fy.front * std::cos(steer) - p.lr * fy.rear;
    const Real m = p.chassis.mass;

    const Real du = (fx / m + s.v * s.r) * dt;
    const Real dv = (fy_total / m - s.u * s.r) * dt;
    const Real dr = mz / p.chassis.inertia.z * dt;
    s.u += du;
    s.v += dv;
    s.r += dr;

    const Real mid = s.yaw + 0.5 * s.r * dt;
    s.x += (s.u * std::cos(mid) - s.v * std::sin(mid)) * dt;
    s.y += (s.u * std::sin(mid) + s.v * std::cos(mid)) * dt;
    s.yaw += s.r * dt;
}

// Largest gap between the rigid-body model and the planar one over the lane change (no speed hold,
// so both are driven by the same script alone).
struct gap
{
    Real pos = 0, yaw = 0, v = 0, r = 0;
};

static gap lane_change_gap(Real dt)
{
    single_track_run run_cfg = single_track_lane_change();
    run_cfg.hold_speed = 0;
    run_cfg.dt = dt;
    run_cfg.steps = std::lround(7.0 / dt);

    rigid_body_state a = run_cfg.initial;
    planar_state b;
    b.u = run_cfg.initial.vel.x;

    gap g;
    for (long i = 0; i < run_cfg.steps; ++i)
    {
        const single_track_input in = sample(run_cfg.script, i * dt);
        step(a, run_cfg.params, in, dt);
        planar_step(b, run_cfg.params, in, dt);

        const body_velocity va = velocity_in_body(a);
        g.pos = std::max(g.pos, std::hypot(a.pos.x - b.x, a.pos.y - b.y));
        g.yaw = std::max(g.yaw, std::fabs(yaw_angle(a.q) - b.yaw));
        g.v = std::max(g.v, std::fabs(va.v - b.v));
        g.r = std::max(g.r, std::fabs(va.r - b.r));
    }
    return g;
}

int main()
{
    const single_track_params p; // 1200 kg, lf = 1.3, lr = 1.2
    const Real weight = p.chassis.mass * kGravity;
    const Real dt = 0.001;

    check_near("wheelbase = lf + lr", p.wheelbase(), 2.5);

    // static axle loads: they carry the whole weight, and the axle nearer the CG carries more
    {
        const axle_pair w = axle_loads(p);
        check_near("loads: front + rear = weight", w.front + w.rear, weight, 1e-9);
        check_near("loads: front = weight lr / L", w.front, weight * 1.2 / 2.5, 1e-9);
        check_true("loads: rear is heavier (CG closer to the rear axle)", w.rear > w.front);
    }

    // velocity in the body frame: the same world velocity is forward or sideways depending on heading
    {
        const body_velocity a = velocity_in_body(car(0.0, 10.0, 0.0, 0.0));
        check_near("body velocity: u at heading 0", a.u, 10.0);
        check_near("body velocity: v at heading 0", a.v, 0.0);

        rigid_body_state s;
        s.q = from_axis_angle(vec3{0, 0, 1}, 3.14159265358979323846 / 2); // facing +y
        s.vel = vec3{10, 0, 0};                                            // moving along world x
        s.omega = vec3{0, 0, 0.3};
        const body_velocity b = velocity_in_body(s);
        check_near("body velocity: moving along world x is sideways to the right", b.v, -10.0);
        check_near("body velocity: forward speed is zero", b.u, 0.0);
        check_near("body velocity: r is omega.z", b.r, 0.3);
    }

    // heading from the quaternion
    for (Real yaw : {0.0, 0.5, 1.5707963267948966, -2.5})
        check_near("yaw_angle of a rotation about z", yaw_angle(from_axis_angle(vec3{0, 0, 1}, yaw)), yaw);

    // slip angles
    {
        const axle_pair straight = slip_angles(p, body_velocity{20, 0, 0}, 0.0);
        check_true("slip: straight ahead, no steer, is zero on both axles", straight.front == 0.0 && straight.rear == 0.0);

        const axle_pair steered = slip_angles(p, body_velocity{20, 0, 0}, 0.1);
        check_near("slip: steer alone gives the front axle that slip angle", steered.front, 0.1);
        check_near("slip: steer alone does not slip the rear axle", steered.rear, 0.0);

        // sliding sideways to the left: both axles move left of where they point, so both slip negative
        const axle_pair sideways = slip_angles(p, body_velocity{20, 2, 0}, 0.0);
        check_near("slip: sideways velocity, front", sideways.front, -std::atan(2.0 / 20.0));
        check_near("slip: sideways velocity, rear", sideways.rear, -std::atan(2.0 / 20.0));

        // turning left (r > 0) at no sideslip: the front axle moves left, the rear moves right
        const axle_pair yawing = slip_angles(p, body_velocity{20, 0, 0.2}, 0.0);
        check_near("slip: yaw rate, front", yawing.front, -std::atan(1.3 * 0.2 / 20.0));
        check_near("slip: yaw rate, rear", yawing.rear, std::atan(1.2 * 0.2 / 20.0));

        // mirror
        const axle_pair m = slip_angles(p, body_velocity{20, -2, -0.2}, -0.1);
        const axle_pair o = slip_angles(p, body_velocity{20, 2, 0.2}, 0.1);
        check_near("slip: mirror, front", m.front, -o.front);
        check_near("slip: mirror, rear", m.rear, -o.rear);

        // below the minimum forward speed the model uses the minimum, so it never divides by zero
        const axle_pair low = slip_angles(p, body_velocity{0.2, 1, 0.3}, 0.1);
        const axle_pair floor = slip_angles(p, body_velocity{kMinForwardSpeed, 1, 0.3}, 0.1);
        check_true("slip: speed below the floor is clamped", low.front == floor.front && low.rear == floor.rear);
        const axle_pair reverse = slip_angles(p, body_velocity{-5, 1, 0.3}, 0.1);
        check_true("slip: reverse is clamped too (placeholder)", reverse.front == floor.front && reverse.rear == floor.rear);
    }

    // lateral forces
    {
        const axle_pair loads = axle_loads(p);

        const axle_pair none = lateral_forces(p, axle_pair{0, 0}, loads);
        check_true("force: zero slip gives zero force", none.front == 0.0 && none.rear == 0.0);

        const axle_pair lin = lateral_forces(p, axle_pair{0.01, -0.02}, loads);
        check_near("force: linear range, front", lin.front, 60000 * 0.01, 1e-9);
        check_near("force: linear range, rear (negative slip, negative force)", lin.rear, 70000 * -0.02, 1e-9);

        const axle_pair sat = lateral_forces(p, axle_pair{0.5, -0.5}, loads);
        check_near("force: front saturates at mu Fz", sat.front, loads.front, 1e-9);
        check_near("force: rear saturates at -mu Fz", sat.rear, -loads.rear, 1e-9);

        // continuous at the limit: the slip angle where C alpha = mu Fz gives exactly mu Fz
        const axle_pair knee = lateral_forces(p, axle_pair{loads.front / 60000, 0}, loads);
        check_near("force: continuous at the friction limit", knee.front, loads.front, 1e-9);

        // grip is mu times load: half the friction coefficient halves the limit
        single_track_params slippery = p;
        slippery.friction_coefficient = 0.5;
        const axle_pair half = lateral_forces(slippery, axle_pair{0.5, 0.5}, loads);
        check_near("force: half the friction coefficient, half the limit", half.front, 0.5 * loads.front, 1e-9);
    }

    // wrench: frames, lever arms and gravity
    {
        // straight at heading 0, drive force only: no torque, gravity cancelled
        const wrench a = single_track_wrench(car(0.0, 20.0, 0.0, 0.0), p, single_track_input{0.0, 500.0});
        check_vec("wrench: straight, force", a.force, vec3{500, 0, weight}, 1e-9);
        check_vec("wrench: straight, torque", a.torque, vec3{0, 0, 0}, 1e-9);

        // heading 0.5 rad: the drive force points along the heading, so it must be rotated to the world frame
        const wrench b = single_track_wrench(car(0.5, 20.0, 0.0, 0.0), p, single_track_input{0.0, 500.0});
        check_vec("wrench: rotated heading, force", b.force,
                  vec3{500 * std::cos(0.5), 500 * std::sin(0.5), weight}, 1e-9);
        check_vec("wrench: rotated heading, torque", b.torque, vec3{0, 0, 0}, 1e-9);

        // a generic state: build the expected force and yaw moment from the axle forces
        const Real yaw = 0.7, steer = 0.05;
        const rigid_body_state s = car(yaw, 20.0, 1.0, 0.3);
        const wrench c = single_track_wrench(s, p, single_track_input{steer, 300.0});
        const axle_pair fy = lateral_forces(p, slip_angles(p, body_velocity{20, 1, 0.3}, steer), axle_loads(p));
        const vec3 force_body{300.0 - fy.front * std::sin(steer), fy.front * std::cos(steer) + fy.rear, 0};
        const vec3 expected_force = to_world(s.q, force_body) + vec3{0, 0, weight};
        check_vec("wrench: general, force", c.force, expected_force, 1e-9);
        check_near("wrench: general, yaw moment = lf Fyf cos(steer) - lr Fyr", c.torque.z,
                   p.lf * fy.front * std::cos(steer) - p.lr * fy.rear, 1e-9);
        check_near("wrench: general, no roll torque", c.torque.x, 0.0, 1e-9);
        check_near("wrench: general, no pitch torque", c.torque.y, 0.0, 1e-9);

        // steering left from a straight line pushes the nose left: positive yaw moment, positive sideways force
        const wrench d = single_track_wrench(car(0.0, 20.0, 0.0, 0.0), p, single_track_input{0.05, 0.0});
        check_true("wrench: steer left gives a positive yaw moment", d.torque.z > 0);
        check_true("wrench: steer left gives a force to the left", d.force.y > 0);
    }

    // straight line with a drive force: constant acceleration F / m, speed updated before position
    {
        rigid_body_state s = single_track_initial(10.0);
        for (int i = 0; i < 1000; ++i)
            step(s, p, single_track_input{0.0, 600.0}, dt);
        const Real a = 600.0 / 1200.0;
        check_near("straight: u = u0 + a t", velocity_in_body(s).u, 10.0 + a * 1.0, 1e-9);
        check_near("straight: x = u0 t + 0.5 a t^2 + 0.5 a t dt", s.pos.x, 10.0 + 0.5 * a + 0.5 * a * dt, 1e-9);
        check_near("straight: y stays zero", s.pos.y, 0.0, 1e-12);
        check_near("straight: heading unchanged", yaw_angle(s.q), 0.0, 1e-12);
    }

    // steering left turns the car left
    {
        rigid_body_state s = single_track_initial(15.0);
        for (int i = 0; i < 1000; ++i)
            step(s, p, single_track_input{0.05, 0.0}, dt);
        check_true("turn: steering left gives a positive yaw rate", velocity_in_body(s).r > 0);
        check_true("turn: steering left moves the car to the left", s.pos.y > 0);
        check_true("turn: heading turns left", yaw_angle(s.q) > 0);
    }

    // the car stays on the ground plane, and a left run mirrors a right run
    {
        single_track_run left = single_track_lane_change();
        single_track_run right = left;
        for (auto &k : right.script.keys)
            k.in.steer = -k.in.steer;
        const rigid_body_state a = run(left);
        const rigid_body_state b = run(right);

        check_near("plane: z", a.pos.z, 0.0, 1e-9);
        check_near("plane: vertical speed", a.vel.z, 0.0, 1e-9);
        check_near("plane: roll rate", a.omega.x, 0.0, 1e-9);
        check_near("plane: pitch rate", a.omega.y, 0.0, 1e-9);

        check_near("mirror: x", a.pos.x, b.pos.x);
        check_near("mirror: y", a.pos.y, -b.pos.y);
        check_near("mirror: heading", yaw_angle(a.q), -yaw_angle(b.q));
        check_near("mirror: sideways speed", a.vel.y, -b.vel.y);
        check_near("mirror: yaw rate", a.omega.z, -b.omega.z);
    }

    // friction: a hard steer at speed drives the front axle into its limit, and no force ever exceeds it
    {
        rigid_body_state s = single_track_initial(20.0);
        const axle_pair loads = axle_loads(p);
        Real peak_front = 0, peak_rear = 0;
        for (int i = 0; i < 2000; ++i)
        {
            const Real steer = applied_steer(p, 0.4);
            const axle_pair fy = lateral_forces(p, slip_angles(p, velocity_in_body(s), steer), loads);
            peak_front = std::max(peak_front, std::fabs(fy.front));
            peak_rear = std::max(peak_rear, std::fabs(fy.rear));
            step(s, p, single_track_input{0.4, 0.0}, dt);
        }
        check_near("friction: front force reaches mu Fz", peak_front, loads.front, 1e-9);
        check_true("friction: rear force never exceeds mu Fz", peak_rear <= loads.rear + 1e-9);
    }

    // steer is limited to max_steer
    {
        rigid_body_state a = single_track_initial(15.0), b = single_track_initial(15.0);
        for (int i = 0; i < 500; ++i)
        {
            step(a, p, single_track_input{p.max_steer, 0.0}, dt);
            step(b, p, single_track_input{p.max_steer + 1.0, 0.0}, dt);
        }
        check_true("steer: more than max_steer acts as max_steer", identical(a, b));
    }

    // steady-state understeer gradient: the steer needed for a circle is L/R + K a_y.
    // Checked for an understeering, a neutral and an oversteering car, at two speeds.
    {
        struct setup
        {
            const char *name;
            Real rear_stiffness;
            int sign; // sign of K
        };
        const setup setups[] = {{"understeer", 70000, 1},
                                {"neutral", 60000 * p.lf / p.lr, 0}, // Wf / Cf = Wr / Cr
                                {"oversteer", 50000, -1}};

        for (const setup &c : setups)
            for (Real speed : {10.0, 20.0})
            {
                single_track_params q = p;
                q.rear_cornering_stiffness = c.rear_stiffness;
                const Real K = understeer_gradient(q);
                const Real steer = (speed < 15) ? 0.03 : 0.015;

                const rigid_body_state s = steady_turn(q, speed, steer, 10000);
                const rigid_body_state later = steady_turn(q, speed, steer, 10100);
                const body_velocity vel = velocity_in_body(s);
                const Real v_cg = std::hypot(vel.u, vel.v);
                const Real radius = v_cg / vel.r;
                const Real a_y = v_cg * vel.r;
                const Real predicted = q.wheelbase() / radius + K * a_y;

                char name[96];
                std::snprintf(name, sizeof name, "%s at %.0f m/s", c.name, speed);
                char what[160];

                std::snprintf(what, sizeof what, "understeer: %s: steer = L/R + K a_y", name);
                check_near(what, steer, predicted, 5e-5);

                std::snprintf(what, sizeof what, "understeer: %s: the turn is steady", name);
                check_near(what, velocity_in_body(later).r, vel.r, 1e-6);

                std::snprintf(what, sizeof what, "understeer: %s: speed is held", name);
                check_near(what, vel.u, speed, 0.05); // a proportional controller settles a little below the target

                // the sign of K decides whether the car needs more or less steer than the geometric angle
                const Real extra = steer - q.wheelbase() / radius;
                std::snprintf(what, sizeof what, "understeer: %s: sign of the extra steer", name);
                if (c.sign > 0)
                    check_true(what, extra > 0);
                else if (c.sign < 0)
                    check_true(what, extra < 0);
                else
                    check_near(what, extra, 0.0, 5e-5);
            }
    }

    // independent planar model: the same equations integrated without the rigid body agree with it,
    // and halving dt halves the gap (both are first order and converge to the same answer)
    {
        const gap coarse = lane_change_gap(0.002);
        const gap fine = lane_change_gap(0.001);
        check_true("planar: position gap below 4 mm over the lane change", fine.pos < 4e-3);
        check_true("planar: heading gap below 3e-5 rad", fine.yaw < 3e-5);
        check_true("planar: sideways speed gap below 1e-3 m/s", fine.v < 1e-3);
        check_true("planar: yaw rate gap below 5e-5 rad/s", fine.r < 5e-5);
        const Real ratio = coarse.pos / fine.pos;
        check_true("planar: halving dt halves the gap", ratio > 1.8 && ratio < 2.2);
    }

    return finish("single_track");
}