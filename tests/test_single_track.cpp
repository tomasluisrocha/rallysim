#include <algorithm>
#include <cmath>
#include <cstdio>
#include "scenarios/single_track_lane_change.hpp"
#include "tests/check.hpp"
#include "tests/single_track_helpers.hpp"

using namespace rallysim;

// Holds `speed` with a constant steer from the start and returns the state after `steps` steps.
static rigid_body_state steady_turn(const single_track_params &p, Real speed, Real steer, long steps)
{
    return run(constant_steer(p, speed, steer, steps));
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

    // load transfer: braking moves load to the front axle, accelerating to the rear, m a_x h / L
    {
        const Real shift = p.chassis.mass * 3.0 * p.cg_height / p.wheelbase(); // at |a_x| = 3 m/s^2
        const axle_pair rest = axle_loads(p, 0.0);
        const axle_pair accel = axle_loads(p, 3.0);
        const axle_pair brake = axle_loads(p, -3.0);

        check_true("transfer: a_x = 0 is the static split", rest.front == axle_loads(p).front && rest.rear == axle_loads(p).rear);
        check_near("transfer: accelerating, the front loses m a_x h / L", accel.front, rest.front - shift, 1e-9);
        check_near("transfer: accelerating, the rear gains the same", accel.rear, rest.rear + shift, 1e-9);
        check_near("transfer: braking, the front gains m a_x h / L", brake.front, rest.front + shift, 1e-9);
        check_near("transfer: braking, the rear loses the same", brake.rear, rest.rear - shift, 1e-9);
        check_near("transfer: the loads always add up to the weight", brake.front + brake.rear, weight, 1e-9);

        // braking at 1 g with the 0.5 m CG: about 2354 N moves to the front
        const axle_pair hard = axle_loads(p, -kGravity);
        check_near("transfer: braking at 1 g moves m g h / L", hard.front - rest.front, p.chassis.mass * kGravity * 0.5 / 2.5, 1e-9);

        // a taller CG moves proportionally more load
        single_track_params tall = p;
        tall.cg_height = 1.0;
        check_near("transfer: twice the CG height, twice the transfer",
                   axle_loads(tall, -3.0).front - rest.front, 2 * shift, 1e-9);

        // the grip follows the load: braking raises the front grip and lowers the rear one
        const axle_pair grip_rest = axle_grip(p, loads_per_wheel(p, 0.0, 0.0));
        const axle_pair grip_brake = axle_grip(p, loads_per_wheel(p, -3.0, 0.0));
        check_true("transfer: braking raises the front grip and lowers the rear one",
                   grip_brake.front > grip_rest.front && grip_brake.rear < grip_rest.rear);

        // an extreme acceleration cannot make an axle load negative: the front lifts and the rear carries
        // the whole car (with a negative load the friction limits would be upside down)
        const axle_pair wheelie = axle_loads(p, 100.0);
        check_near("transfer: wheelie, the front axle carries nothing", wheelie.front, 0.0, 1e-9);
        check_near("transfer: wheelie, the rear axle carries the weight", wheelie.rear, weight, 1e-9);
        const axle_pair stoppie = axle_loads(p, -100.0);
        check_near("transfer: stoppie, the rear axle carries nothing", stoppie.rear, 0.0, 1e-9);
        check_near("transfer: stoppie, the front axle carries the weight", stoppie.front, weight, 1e-9);
    }

    // load sensitivity: the friction coefficient falls as the load rises, so an axle with its load
    // shared unevenly between its wheels has less grip than with it shared evenly
    {
        const Real mu0 = p.friction_coefficient, k = p.load_sensitivity, fz0 = p.nominal_wheel_load;
        check_near("sensitivity: mu at the nominal load is mu0", peak_friction(p, fz0), mu0, 1e-12);
        check_near("sensitivity: mu at twice the nominal load", peak_friction(p, 2 * fz0), mu0 * (1 - k), 1e-12);
        check_near("sensitivity: mu at no load", peak_friction(p, 0.0), mu0 * (1 + k), 1e-12);
        check_near("sensitivity: mu never goes below zero", peak_friction(p, 100 * fz0), 0.0, 1e-12);

        // grip of one wheel is mu(fz) fz = mu0 (fz (1 + k) - k fz^2 / fz0); moving d from one wheel to the
        // other of an axle loses 2 mu0 k d^2 / fz0 of grip
        const Real d = 1000.0;
        const wheel_loads even{3000, 3000, 3000, 3000};
        const wheel_loads uneven{3000 - d, 3000 + d, 3000, 3000};
        const axle_pair g_even = axle_grip(p, even), g_uneven = axle_grip(p, uneven);
        check_near("sensitivity: grip of an evenly loaded axle", g_even.front, 2 * mu0 * fz0, 1e-9);
        check_near("sensitivity: moving d between the wheels loses 2 mu0 k d^2 / fz0",
                   g_even.front - g_uneven.front, 2 * mu0 * k * d * d / fz0, 1e-9);
        check_near("sensitivity: the other axle is untouched", g_uneven.rear, g_even.rear, 1e-12);

        // with no sensitivity the grip is mu times the load, however it is shared
        single_track_params flat = p;
        flat.load_sensitivity = 0;
        check_near("sensitivity off: grip = mu Fz", axle_grip(flat, uneven).front, 6000.0, 1e-9);
        flat.friction_coefficient = 0.5;
        check_near("sensitivity off: half the friction coefficient, half the grip", axle_grip(flat, uneven).front, 3000.0, 1e-9);

        // the side that takes more of the lateral transfer loses more grip: in a turn, a front share of 1
        // leaves the front axle with less grip and the rear with more than a share of 0
        single_track_params front_heavy = p, rear_heavy = p;
        front_heavy.front_transfer_share = 1.0;
        rear_heavy.front_transfer_share = 0.0;
        const axle_pair gf = axle_grip(front_heavy, loads_per_wheel(front_heavy, 0.0, 6.0));
        const axle_pair gr = axle_grip(rear_heavy, loads_per_wheel(rear_heavy, 0.0, 6.0));
        check_true("sensitivity: more transfer on the front, less front grip", gf.front < gr.front);
        check_true("sensitivity: and more rear grip", gf.rear > gr.rear);
    }

    // traction limit of the rear-drive axle: F = mu (W lf / L + F h / L) at the limit
    {
        const Real mu = p.friction_coefficient, L = p.wheelbase(), h = p.cg_height;
        const Real forward = mu * weight * p.lf / (L - mu * h);
        const Real backward = mu * weight * p.lf / (L + mu * h);
        check_near("traction: a small force is passed unchanged", traction_limited_force(p, 500.0), 500.0, 1e-12);
        check_near("traction: accelerating, limit mu W lf / (L - mu h)", traction_limited_force(p, 1e6), forward, 1e-9);
        check_near("traction: braking, limit mu W lf / (L + mu h)", traction_limited_force(p, -1e6), -backward, 1e-9);
        check_near("traction: at the limit the force equals mu times the rear load it causes",
                   forward, mu * axle_loads(p, forward / p.chassis.mass).rear, 1e-9);
        check_near("traction: a_x estimate uses the limited force",
                   longitudinal_acceleration(p, single_track_input{0.0, 1e6}), forward / p.chassis.mass, 1e-12);
    }

    // lateral acceleration estimate: forward speed times yaw rate, positive when turning left
    {
        check_near("a_y: u * r", lateral_acceleration(body_velocity{20, 0.5, 0.2}), 4.0);
        check_near("a_y: negative when turning right", lateral_acceleration(body_velocity{20, 0, -0.2}), -4.0);
        check_near("a_y: zero on a straight", lateral_acceleration(body_velocity{20, 0, 0}), 0.0);
    }

    // loads per wheel: the longitudinal transfer between the axles, then the lateral transfer between the sides
    {
        const Real w = weight;
        const auto sum = [](const wheel_loads &l)
        { return l.front_left + l.front_right + l.rear_left + l.rear_right; };

        // no acceleration: static loads, left and right equal
        const wheel_loads rest = loads_per_wheel(p, 0.0, 0.0);
        check_near("wheels: at rest the front wheels are equal", rest.front_left, rest.front_right);
        check_near("wheels: at rest the rear wheels are equal", rest.rear_left, rest.rear_right);
        check_near("wheels: at rest each front wheel carries half the front axle", rest.front_left, axle_loads(p).front / 2, 1e-9);
        check_near("wheels: at rest the loads add up to the weight", sum(rest), w, 1e-9);

        // a left turn at a_y = 4: m a_y h / t = 1600 N goes from the left side to the right, half through each axle
        const Real total = p.chassis.mass * 4.0 * p.cg_height / p.track_width;
        const wheel_loads left = loads_per_wheel(p, 0.0, 4.0);
        check_near("wheels: left turn, front left loses its share", left.front_left, rest.front_left - 0.5 * total, 1e-9);
        check_near("wheels: left turn, front right gains it", left.front_right, rest.front_right + 0.5 * total, 1e-9);
        check_near("wheels: left turn, rear left loses its share", left.rear_left, rest.rear_left - 0.5 * total, 1e-9);
        check_near("wheels: left turn, rear right gains it", left.rear_right, rest.rear_right + 0.5 * total, 1e-9);
        check_near("wheels: left turn, the right side is heavier by 2 m a_y h / t",
                   (left.front_right + left.rear_right) - (left.front_left + left.rear_left), 2 * total, 1e-9);
        check_near("wheels: left turn, the loads add up to the weight", sum(left), w, 1e-9);

        // right turn mirrors it
        const wheel_loads right = loads_per_wheel(p, 0.0, -4.0);
        check_near("wheels: mirror, front", right.front_left, left.front_right, 1e-9);
        check_near("wheels: mirror, front other side", right.front_right, left.front_left, 1e-9);
        check_near("wheels: mirror, rear", right.rear_left, left.rear_right, 1e-9);
        check_near("wheels: mirror, rear other side", right.rear_right, left.rear_left, 1e-9);

        // the front share decides how the transfer is split between the axles
        single_track_params front_only = p, rear_only = p;
        front_only.front_transfer_share = 1.0;
        rear_only.front_transfer_share = 0.0;
        const wheel_loads f = loads_per_wheel(front_only, 0.0, 4.0);
        check_near("wheels: share 1, the rear wheels stay equal", f.rear_left, f.rear_right, 1e-9);
        check_near("wheels: share 1, the front takes all of it", f.front_right - f.front_left, 2 * total, 1e-9);
        const wheel_loads r = loads_per_wheel(rear_only, 0.0, 4.0);
        check_near("wheels: share 0, the front wheels stay equal", r.front_left, r.front_right, 1e-9);
        check_near("wheels: share 0, the rear takes all of it", r.rear_right - r.rear_left, 2 * total, 1e-9);

        // a taller CG or a narrower track moves proportionally more
        single_track_params tall = p, narrow = p;
        tall.cg_height = 1.0;
        narrow.track_width = 0.75;
        check_near("wheels: twice the CG height, twice the transfer",
                   loads_per_wheel(tall, 0.0, 4.0).front_right - rest.front_right, 2 * 0.5 * total, 1e-9);
        check_near("wheels: half the track, twice the transfer",
                   loads_per_wheel(narrow, 0.0, 4.0).front_right - rest.front_right, 2 * 0.5 * total, 1e-9);

        // braking and cornering together: each axle keeps the load it has with the longitudinal transfer
        const wheel_loads both = loads_per_wheel(p, -5.0, 4.0);
        const axle_pair axle = axle_loads(p, -5.0);
        check_near("wheels: braking in a turn, the front axle load", both.front_left + both.front_right, axle.front, 1e-9);
        check_near("wheels: braking in a turn, the rear axle load", both.rear_left + both.rear_right, axle.rear, 1e-9);
        check_near("wheels: braking in a turn, the loads add up to the weight", sum(both), w, 1e-9);

        // a lot of lateral acceleration lifts the inner front wheel (3000 N of transfer on a 2825 N half axle);
        // the outer wheel carries the whole axle, the weight is kept, and no wheel goes negative
        const wheel_loads lift = loads_per_wheel(p, 0.0, 15.0);
        check_near("wheels: lifted inner front wheel carries nothing", lift.front_left, 0.0, 1e-9);
        check_near("wheels: the outer front wheel carries the whole axle", lift.front_right, axle_loads(p).front, 1e-9);
        check_true("wheels: the rear inner wheel has not lifted at this acceleration", lift.rear_left > 0);
        check_near("wheels: lift keeps the weight", sum(lift), w, 1e-9);
        check_near("wheels: the rear axle is untouched by the front lift", lift.rear_right - lift.rear_left, 2 * 0.5 * (p.chassis.mass * 15.0 * p.cg_height / p.track_width), 1e-9);
    }

    // velocity in the body frame: the same world velocity is forward or sideways depending on heading
    {
        const body_velocity a = velocity_in_body(car(0.0, 10.0, 0.0, 0.0));
        check_near("body velocity: u at heading 0", a.u, 10.0);
        check_near("body velocity: v at heading 0", a.v, 0.0);

        rigid_body_state s;
        s.q = from_axis_angle(vec3{0, 0, 1}, kPi / 2); // facing +y
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
        const axle_pair grip = axle_grip(p, loads_per_wheel(p, 0.0, 0.0));

        const axle_pair none = lateral_forces(p, axle_pair{0, 0}, grip);
        check_true("force: zero slip gives zero force", none.front == 0.0 && none.rear == 0.0);

        const axle_pair lin = lateral_forces(p, axle_pair{0.01, -0.02}, grip);
        check_near("force: linear range, front", lin.front, 60000 * 0.01, 1e-9);
        check_near("force: linear range, rear (negative slip, negative force)", lin.rear, 70000 * -0.02, 1e-9);

        const axle_pair sat = lateral_forces(p, axle_pair{0.5, -0.5}, grip);
        check_near("force: front saturates at its grip", sat.front, grip.front, 1e-9);
        check_near("force: rear saturates at minus its grip", sat.rear, -grip.rear, 1e-9);

        // continuous at the limit: the slip angle where C alpha = grip gives exactly the grip
        const axle_pair knee = lateral_forces(p, axle_pair{grip.front / 60000, 0}, grip);
        check_near("force: continuous at the friction limit", knee.front, grip.front, 1e-9);
    }

    // evaluate: the rear axle shares its grip between the drive force and the side force
    {
        const rigid_body_state s = car(0.0, 20.0, -2.5, 0.0); // sliding right: both axles saturate
        const single_track_forces plain = evaluate(s, p, single_track_input{0.0, 0.0});
        const single_track_forces driven = evaluate(s, p, single_track_input{0.0, 3000.0});
        check_near("circle: with no drive the rear side force is the whole grip", plain.fy.rear, plain.grip.rear, 1e-9);
        check_near("circle: the drive force passes unchanged below the grip", driven.fx_rear, 3000.0, 1e-12);
        check_near("circle: Fx^2 + Fy^2 = grip^2 on the rear axle",
                   driven.fx_rear * driven.fx_rear + driven.fy.rear * driven.fy.rear,
                   driven.grip.rear * driven.grip.rear, 1e-6);

        // a drive force beyond the grip is cut to the grip, and leaves nothing for the side force
        const single_track_forces spin = evaluate(s, p, single_track_input{0.0, 1e6});
        check_near("circle: the drive force is limited by the rear grip", spin.fx_rear, spin.grip.rear, 1e-9);
        check_near("circle: wheelspin leaves no side grip", spin.fy.rear, 0.0, 1e-9);
        const single_track_forces lock = evaluate(s, p, single_track_input{0.0, -1e6});
        check_near("circle: so is a braking force", lock.fx_rear, -lock.grip.rear, 1e-9);
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

        // braking: the wrench uses the loads with transfer. Both axles are saturated here (sideways speed
        // of -2.5 m/s at 20 m/s), so the force on the body is the sum of the limits at those loads; the
        // rear axle also brakes, so its side force is what the braking force leaves (friction circle).
        {
            const Real steer2 = 0.5, drive = -3000.0;
            const rigid_body_state t = car(0.0, 20.0, -2.5, 0.0);
            const wrench e = single_track_wrench(t, p, single_track_input{steer2, drive});
            const axle_pair grip = axle_grip(p, loads_per_wheel(p, drive / p.chassis.mass, 0.0));
            const Real rear_side = std::sqrt(grip.rear * grip.rear - drive * drive);
            check_true("wrench: braking raises the front grip", grip.front > axle_grip(p, loads_per_wheel(p, 0, 0)).front);
            check_near("wrench: braking, force along the car = braking force - Fyf sin(steer)", to_body(t.q, e.force).x,
                       drive - grip.front * std::sin(steer2), 1e-9);
            check_near("wrench: braking, sideways force from the shifted loads", e.force.y,
                       grip.front * std::cos(steer2) + rear_side, 1e-9);
            check_near("wrench: braking, yaw moment from the shifted loads", e.torque.z,
                       p.lf * grip.front * std::cos(steer2) - p.lr * rear_side, 1e-9);
        }

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
            k.value.steer = -k.value.steer;
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

    // friction: a hard steer at speed drives the front axle into its limit, and no force ever exceeds the
    // grip it has at that moment (with load transfer and load sensitivity)
    {
        rigid_body_state s = single_track_initial(20.0);
        bool front_reached = false, within = true;
        for (int i = 0; i < 2000; ++i)
        {
            const single_track_input in{0.4, 400.0};
            const single_track_forces f = evaluate(s, p, in);
            front_reached = front_reached || std::fabs(f.fy.front) >= f.grip.front - 1e-9;
            within = within && std::fabs(f.fy.front) <= f.grip.front + 1e-9 &&
                     std::hypot(f.fx_rear, f.fy.rear) <= f.grip.rear + 1e-6;
            step(s, p, in, dt);
        }
        check_true("friction: the front force reaches the front grip", front_reached);
        check_true("friction: no axle force ever exceeds its grip", within);
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