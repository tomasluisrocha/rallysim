#include <cmath>
#include <cstdio>
#include <vector>
#include "tests/check.hpp"
#include "runner/run_four_wheel.hpp"
#include "vehicle/four_wheel.hpp"

using namespace rallysim;

// Wheel travel as each corner's coordinate and the motion ratio between it and the strut (unsprung model). Each
// check is against a hand calculation or a closed form: the car at rest, the wheel rate k MR^2, the damper's c MR^2,
// and the two-mass heave frequency.

namespace
{
    // The unsprung car with motion ratio `mr` and gain `gain` on both axles.
    four_wheel_params geometry_car(Real mr, Real gain)
    {
        four_wheel_params p;
        p.vertical = vertical_model::unsprung;
        p.front_corner.motion_ratio = p.rear_corner.motion_ratio = mr;
        p.front_corner.motion_ratio_gain = p.rear_corner.motion_ratio_gain = gain;
        return p;
    }

    // FL's strut force at the wheel with its wheel moved `dz` from rest, the body at rest.
    Real fl_strut_at(const four_wheel_params &p, Real dz)
    {
        four_wheel_state s = four_wheel_initial(p, 0.0);
        s.corners[FL].z += dz;
        return evaluate(s, p, four_wheel_input{}).corner[FL].strut;
    }

    Real fl_wheel_rate(const four_wheel_params &p, Real dz)
    {
        const Real h = 1e-6;
        return (fl_strut_at(p, dz + h) - fl_strut_at(p, dz - h)) / (2 * h);
    }
}

int main()
{
    const Real g = kGravity;
    const four_wheel_input none{};

    // at rest, MR 0.7: the strut is compressed x0 = (Fz0 - mu g) / (k MR) and pushes the wheel with k x0 MR, the
    // tyre load less the wheel's weight; the car stays level at cg_height
    {
        const four_wheel_params p = geometry_car(0.7, 0);
        four_wheel_state s = four_wheel_initial(p, 0.0);
        const four_wheel_forces f = evaluate(s, p, none);
        const Real x0 = (2825.28 - 40 * g) / (50000 * 0.7);
        std::printf("rest, MR 0.7: tyre %.6f N, strut at the wheel %.6f N, spring %.6f N, deflection %.9f m (x0 %.9f)\n",
                    f.fz[FL], f.corner[FL].strut, f.corner[FL].spring, f.corner[FL].deflection, x0);
        check_near("rest: FL tyre load", f.fz[FL], 2825.28, 1e-6);
        check_near("rest: FL strut force at the wheel", f.corner[FL].strut, 2432.88, 1e-6);
        check_near("rest: FL spring", f.corner[FL].spring, 3475.542857, 1e-6);
        check_near("rest: FL strut deflection", f.corner[FL].deflection, 0.069510857, 1e-9);
        check_near("rest: FL strut deflection = x0", p.strut_static_deflection(FL), x0, 1e-15);

        for (int i = 0; i < 10000; ++i)
            step(s, p, none, 0.001);
        std::printf("rest after 10 s, MR 0.7: z %.12f, pitch %.3g, roll %.3g\n", s.body.pos.z, pitch_angle(s.body.q),
                    roll_angle(s.body.q));
        check_near("rest: still at cg_height after 10 s", s.body.pos.z, p.cg_height, 1e-9);
        check_near("rest: no pitch", pitch_angle(s.body.q), 0.0, 1e-9);
        check_near("rest: no roll", roll_angle(s.body.q), 0.0, 1e-9);
    }

    // wheel rate: the strut force at the wheel grows by k MR^2 per metre of wheel travel
    {
        const Real rate = fl_wheel_rate(geometry_car(0.7, 0), 0);
        std::printf("wheel rate, MR 0.7: %.4f N/m\n", rate);
        check_near("wheel rate: k MR^2", rate, 24500, 0.01);
    }

    // with a gain of 2 /m: k MR^2 + gain k x0 at rest; with FL raised 0.02 m, MR 0.74 and
    // x = x0 + 0.7 x 0.02 + 0.02^2, so the force at the wheel is k x MR
    {
        const four_wheel_params p = geometry_car(0.7, 2);
        const Real rate = fl_wheel_rate(p, 0), raised = fl_strut_at(p, 0.02);
        std::printf("gain 2: wheel rate %.4f N/m, strut force at the wheel raised 20 mm %.4f N\n", rate, raised);
        check_near("gain: wheel rate k MR^2 + gain k x0", rate, 31451.0857, 0.01);
        check_near("gain: strut force at the wheel raised 20 mm", raised, 3104.7017, 1e-3);
    }

    // damper: the wheel moving up at 0.1 m/s compresses the strut at MR v, and the force at the wheel is c MR^2 v
    {
        const four_wheel_params p = geometry_car(0.7, 0);
        four_wheel_state s = four_wheel_initial(p, 0.0);
        const Real still = evaluate(s, p, none).corner[FL].strut;
        s.corners[FL].vz = 0.1;
        const four_wheel_forces f = evaluate(s, p, none);
        std::printf("damper: strut force at the wheel changes %.6f N, damper %.6f N\n", f.corner[FL].strut - still,
                    f.corner[FL].damper);
        check_near("damper: force at the wheel c MR^2 v", f.corner[FL].strut - still, 186.2, 1e-9);
        check_near("damper: the strut's own damper c MR v", f.corner[FL].damper, 266.0, 1e-9);
    }

    // anti-roll bar, MR 0.7: the bar acts on the wheel travel, 20000 x 0.02 = 400 N with FL raised 0.02 m.
    // Catches the bar on the strut compression (280 N).
    {
        four_wheel_params p = geometry_car(0.7, 0);
        p.front_corner.antiroll_rate = 20000;
        four_wheel_state s = four_wheel_initial(p, 0.0);
        s.corners[FL].z += 0.02;
        const four_wheel_forces f = evaluate(s, p, none);
        std::printf("anti-roll, MR 0.7: FL %.6f N, FR %.6f N\n", f.corner[FL].antiroll, f.corner[FR].antiroll);
        check_near("anti-roll: FL bar force on wheel travel", f.corner[FL].antiroll, 400, 1e-9);
        check_near("anti-roll: FR bar force on wheel travel", f.corner[FR].antiroll, -400, 1e-9);
    }

    // heave: four equal corners, no dampers; the body lifted 5 mm rings at the body mode of the two-mass system
    // with wheel rate kw = k MR^2, tyre kt, a quarter of the sprung mass and the wheel's mass
    {
        four_wheel_params p = geometry_car(0.7, 0);
        p.lf = p.lr = 1.25;
        p.front_corner.damper_rate = p.rear_corner.damper_rate = 0;
        rebuild_tyres(p);
        four_wheel_state s = four_wheel_initial(p, 0.0);
        s.body.pos.z += 0.005;
        const Real dt = 0.0001;
        std::vector<Real> crossings;
        Real prev = s.body.pos.z - p.cg_height;
        for (long i = 1; i <= 100000; ++i)
        {
            step(s, p, none, dt);
            const Real now = s.body.pos.z - p.cg_height;
            if (prev < 0 && now >= 0)
                crossings.push_back((i - 1 + prev / (prev - now)) * dt);
            prev = now;
        }
        const Real kw = 50000 * 0.49, kt = 250000, ms = p.sprung_mass() / 4, mu = 40;
        const Real b = kw * mu + (kw + kt) * ms;
        const Real lambda = (b - std::sqrt(b * b - 4 * ms * mu * kw * kt)) / (2 * ms * mu);
        const Real expected = std::sqrt(lambda) / (2 * std::acos(-1.0));
        check_true("heave: enough crossings", crossings.size() > 10);
        if (crossings.size() >= 2)
        {
            const Real measured = (crossings.size() - 1) / (crossings.back() - crossings.front());
            std::printf("heave, MR 0.7: %.5f Hz over %zu crossings, two-mass closed form %.5f Hz\n", measured,
                        crossings.size(), expected);
            check_near("heave: frequency within 0.1% of the two-mass closed form", measured / expected, 1.0, 1e-3);
        }
        else
            check_true("heave: frequency within 0.1% of the two-mass closed form (fewer than two crossings)", false);
        check_near("heave: closed form", expected, 1.47349, 1e-5);
    }

    // defaults: a ratio of 1 and no gain leave the strut exactly the old expression
    {
        four_wheel_params p;
        p.vertical = vertical_model::unsprung;
        four_wheel_pose pose;
        pose.position = {0, 0, p.cg_height};
        pose.roll = 0.01;
        pose.velocity = {0, 0, 0.03};
        pose.angular_velocity = {0.1, 0, 0};
        four_wheel_state s = four_wheel_start(p, pose);
        s.corners[FL].z += 0.013;
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            const contact_geometry c = contact_of(p, s.body, i, 0.0);
            const Real mount = p.static_deflection(i) - c.deflection, mount_rate = -c.deflection_rate;
            const corner_state &w = s.corners[i];
            const corner_kinematics k = kinematics_of(p, i, mount, mount_rate, w);
            check_true("defaults: ratio is exactly 1", k.ratio == 1);
            check_true("defaults: strut is exactly x0 - mount + (z - zw0)",
                       k.strut == p.strut_static_deflection(i) - mount + (w.z - p.wheel_rest_height(i)));
            check_true("defaults: strut rate is exactly vz - mount rate", k.strut_rate == w.vz - mount_rate);
        }
    }

    // Ackermann: each front wheel's angle is atan(L / (R -+ t/2)) on the circle of radius R = L / tan(steer), the
    // inner wheel turning more. Catches the two wheels swapped.
    {
        four_wheel_params p;
        p.ackermann = 1;
        const Real L = p.wheelbase(), t = p.track_front;
        const Real cases[][3] = {{0.125, 0.129842820, 0.120503753}, {-0.2, -0.188813101, -0.212574753}};
        for (const auto &c : cases)
        {
            const Real fl = wheel_steer(p, FL, c[0], 0), fr = wheel_steer(p, FR, c[0], 0);
            const Real R = L / std::tan(c[0]);
            std::printf("ackermann 1, steer %.3f: FL %.9f, FR %.9f\n", c[0], fl, fr);
            check_near("ackermann: FL", fl, c[1], 1e-9);
            check_near("ackermann: FR", fr, c[2], 1e-9);
            check_near("ackermann: FL is atan(L / (R - t/2))", fl, std::atan(L / (R - t / 2)), 1e-9);
            check_near("ackermann: FR is atan(L / (R + t/2))", fr, std::atan(L / (R + t / 2)), 1e-9);
            check_near("ackermann: rear wheels straight", wheel_steer(p, RL, c[0], 0), 0.0, 0.0);
        }
        p.ackermann = 0.5;
        check_near("ackermann 0.5: FL", wheel_steer(p, FL, 0.3, 0), 0.314308097, 1e-9);
        check_near("ackermann 0.5: FR", wheel_steer(p, FR, 0.3, 0), 0.287925393, 1e-9);
        p.ackermann = 0;
        check_true("ackermann 0: both wheels turn by the steer exactly",
                   wheel_steer(p, FL, 0.3, 0) == 0.3 && wheel_steer(p, FR, 0.3, 0) == 0.3);
    }

    // a slow circle at 5 m/s, steer 0.125: with Ackermann 1 the path radius is the geometric L / tan(steer) and the
    // front slip angles agree; with 0 the outer wheel is turned too much and the slips differ. Slower still, at
    // 1 m/s, the tyres hardly slip, so with 0 the two front tyres push against each other and with 1 they do not.
    // Catches a reversed split (the slips would differ twice as much).
    {
        auto circle = [](Real ackermann, Real speed, four_wheel_state &s, four_wheel_params &p)
        {
            p = geometry_car(1, 0);
            p.ackermann = ackermann;
            four_wheel_run r;
            r.params = p;
            r.initial = four_wheel_initial(p, speed);
            r.hold_speed = constant_script<Real>(speed);
            r.script.keys = {{0.0, four_wheel_input{0.125, 0.0, 0.0, 0.0}}};
            r.steps = 20000;
            s = run(r);
            return r;
        };
        auto slip = [](const four_wheel_state &s, const four_wheel_params &, std::size_t i)
        { return std::fabs(std::atan(s.wheels[i].lateral_slip)); };
        four_wheel_state s;
        four_wheel_params p;
        circle(1, 5.0, s, p);
        const Real radius = std::hypot(velocity_in_body(s.body).u, velocity_in_body(s.body).v) / s.body.omega.z;
        const Real expected = p.wheelbase() / std::tan(0.125);
        const Real d1 = std::fabs(slip(s, p, FL) - slip(s, p, FR));
        std::printf("circle, ackermann 1: radius %.4f m (geometric %.4f), front slips %.5f %.5f\n", radius, expected,
                    slip(s, p, FL), slip(s, p, FR));
        check_near("circle: radius within 1.5% of L / tan(steer)", radius, expected, 0.015 * expected);
        check_true("circle: ackermann 1 front slips within 0.0015 rad", d1 < 0.0015);
        circle(0, 5.0, s, p);
        const Real d0 = std::fabs(slip(s, p, FL) - slip(s, p, FR));
        std::printf("circle, ackermann 0: front slips %.5f %.5f\n", slip(s, p, FL), slip(s, p, FR));
        check_true("circle: ackermann 0 front slips differ by more than 0.007 rad", d0 > 0.007);

        for (Real a : {0.0, 1.0})
        {
            const four_wheel_run r = circle(a, 1.0, s, p);
            const four_wheel_forces f = evaluate(s, p, driver_input(r, s, r.steps * r.dt));
            std::printf("1 m/s, ackermann %.0f: Fy FL %.1f N, FR %.1f N\n", a, f.tyre[FL].fy, f.tyre[FR].fy);
            if (a == 0)
                check_true("slow: ackermann 0 tyres push against each other",
                           f.tyre[FL].fy < -100 && f.tyre[FR].fy > 100);
            else
                check_true("slow: ackermann 1 both tyres within 20 N of 15 N",
                           std::fabs(f.tyre[FL].fy - 15) < 20 && std::fabs(f.tyre[FR].fy - 15) < 20);
        }
    }

    // toe-in 0.004 rad on both axles, linear tyres (C = 30000), straight at 20 m/s: each tyre pushes inboard with
    // C tan(toe) and the pairs cancel, so the car goes straight. Catches toe-out read as toe-in.
    {
        four_wheel_params p = geometry_car(1, 0);
        p.set_tyre_curve(tyre_curve::linear);
        p.front_tyre.cornering_stiffness = p.rear_tyre.cornering_stiffness = 30000;
        p.lf = p.lr = 1.25;
        rebuild_tyres(p);
        p.front_corner.toe = p.rear_corner.toe = 0.004;
        four_wheel_run r;
        r.params = p;
        r.initial = four_wheel_initial(p, 20.0);
        r.hold_speed = constant_script<Real>(20.0);
        r.script.keys = {{0.0, four_wheel_input{0.0, 0.0, 0.0, 0.0}}};
        r.steps = 6000;
        const four_wheel_state s = run(r);
        const four_wheel_forces f = evaluate(s, p, driver_input(r, s, r.steps * r.dt));
        const Real expected = 30000 * std::tan(0.004);
        std::printf("toe-in: Fy FL %.4f RL %.4f FR %.4f RR %.4f N (C tan(toe) %.4f), y %.3g m\n", f.tyre[FL].fy,
                    f.tyre[RL].fy, f.tyre[FR].fy, f.tyre[RR].fy, expected, s.body.pos.y);
        check_near("toe: FL pushes right", f.tyre[FL].fy, -expected, 0.01);
        check_near("toe: RL pushes right", f.tyre[RL].fy, -expected, 0.01);
        check_near("toe: FR pushes left", f.tyre[FR].fy, expected, 0.01);
        check_near("toe: RR pushes left", f.tyre[RR].fy, expected, 0.01);
        check_near("toe: the car has not moved sideways", s.body.pos.y, 0.0, 1e-9);
    }

    // the wrench takes each tyre force along the wheel's own angle: 1000 N more sideways force on FL, toed in 0.004
    // rad, pushes the body 1000 sin(toe) forward and 1000 cos(toe) left. Catches four_wheel_wrench using the bare
    // steer (the heading), which gives no forward part.
    {
        four_wheel_params p = geometry_car(1, 0);
        p.front_corner.toe = 0.004;
        const four_wheel_state s = four_wheel_initial(p, 0.0);
        const four_wheel_forces f = evaluate(s, p, none);
        four_wheel_forces more = f;
        more.tyre[FL].fy += 1000;
        const vec3 d = four_wheel_wrench(s, p, more).force - four_wheel_wrench(s, p, f).force;
        std::printf("wrench: FL +1000 N sideways moves the body by x %.6f, y %.6f N\n", d.x, d.y);
        check_near("wrench: forward part 1000 sin(toe)", d.x, 1000 * std::sin(0.004), 1e-9);
        check_near("wrench: left part 1000 cos(toe)", d.y, 1000 * std::cos(0.004), 1e-9);
    }

    // toe against wheel travel: toe_gain 0.5 on the front, FL raised 0.02 m: 0.01 rad of toe-in on FL, which is
    // 0.01 rad right of straight
    {
        four_wheel_params p = geometry_car(1, 0);
        p.front_corner.toe_gain = 0.5;
        four_wheel_state s = four_wheel_initial(p, 0.0);
        s.corners[FL].z += 0.02;
        const four_wheel_forces f = evaluate(s, p, none);
        std::printf("toe gain: FL toe %.6f steer %.6f, FR toe %.6f\n", f.kin[FL].toe, f.kin[FL].steer, f.kin[FR].toe);
        check_near("toe gain: FL toe", f.kin[FL].toe, 0.01, 1e-12);
        check_near("toe gain: FR toe", f.kin[FR].toe, 0.0, 1e-12);
        check_near("toe gain: FL steer", f.kin[FL].steer, -0.01, 1e-12);
    }

    // roll centre 0.15 m on the front, at rest: 1000 N to the left on each front tyre gives a jacking force of
    // slope x force (slope 0.15 / 0.75), and the body's roll torque changes by the 2000 N acting at the roll centre,
    // 0.35 m below the CG. Catches the sign (1300 N m) and a missing jacking term (1000 N m).
    {
        four_wheel_params p = geometry_car(1, 0);
        p.front_corner.roll_centre = 0.15;
        const four_wheel_state s = four_wheel_initial(p, 0.0);
        const four_wheel_forces f = evaluate(s, p, none);
        four_wheel_forces pushed = f;
        pushed.tyre[FL].fy += 1000;
        pushed.tyre[FR].fy += 1000;
        set_jacking(pushed, s, p);
        const Real roll = four_wheel_wrench(s, p, pushed).torque.x - four_wheel_wrench(s, p, f).torque.x;
        std::printf("roll centre 0.15: jacking FL %.6f N, FR %.6f N, roll torque change %.6f N m\n", pushed.corner[FL].jacking,
                    pushed.corner[FR].jacking, roll);
        check_near("roll centre: FL jacking", pushed.corner[FL].jacking, -200, 1e-9);
        check_near("roll centre: FR jacking", pushed.corner[FR].jacking, 200, 1e-9);
        check_near("roll centre: roll torque of the force at the roll centre", roll, 700, 1e-6);
    }

    // the patch's sideways path: FL raised 0.02 m with roll centre 0.15 and gain -0.5 moves out by
    // (0.15 x 0.02 - 0.25 x 0.02^2) / 0.75; moving up at 0.1 m/s with no gain, the patch moves outboard at slope x 0.1.
    // Catches a missing sideways speed and a wrong side.
    {
        four_wheel_params p = geometry_car(1, 0);
        p.front_corner.roll_centre = 0.15;
        p.front_corner.roll_centre_gain = -0.5;
        four_wheel_state s = four_wheel_initial(p, 0.0);
        s.corners[FL].z += 0.02;
        const contact_geometry c = contact_of(p, s.body, FL, 0.0, &s.corners[FL]);
        std::printf("roll centre path: FL patch out by %.9f m\n", c.point.y - c.load_point.y);
        check_near("roll centre path: FL patch moves outboard", c.point.y - c.load_point.y, 0.0029 / 0.75, 1e-9);
        check_near("roll centre path: FL patch outboard value", c.point.y - c.load_point.y, 0.00386667, 1e-8);

        p.front_corner.roll_centre_gain = 0;
        four_wheel_state m = four_wheel_initial(p, 0.0);
        m.corners[FL].vz = m.corners[FR].vz = 0.1;
        const Real fl = contact_of(p, m.body, FL, 0.0, &m.corners[FL]).velocity.vy;
        const Real fr = contact_of(p, m.body, FR, 0.0, &m.corners[FR]).velocity.vy;
        std::printf("roll centre speed: FL patch %.12f m/s, FR %.12f m/s across the wheel\n", fl, fr);
        check_near("roll centre speed: FL patch moves left", fl, 0.02, 1e-12);
        check_near("roll centre speed: FR patch moves right", fr, -0.02, 1e-12);
    }

    // steady left turn, 20 m/s held, roll centres on both axles. Per axle, the roll stiffness at the ground is
    // K = kw kt / (kw + kt) track^2 / 2 and the linkage carries G = kt / (kt + kw) h F of the moment, F the axle's
    // force along the body's left. Then roll = (m a_y hz - G_f - G_r) / (K_f + K_r - ms g hz), hz the CG's height,
    // and each axle's transfer moment is K roll + G. Catches jacking on the body only (the loads would not add up),
    // the geometric moment at full strength (roll 5.9% off) and, with the ratio 0.8, a wheel rate of k MR in roll.
    {
        struct turn
        {
            Real roll, expected, share, expected_share;
        };
        auto steady = [&](Real front, Real rear, Real mr)
        {
            four_wheel_params p = geometry_car(mr, 0);
            p.front_corner.roll_centre = front;
            p.rear_corner.roll_centre = rear;
            four_wheel_run r;
            r.params = p;
            r.initial = four_wheel_initial(p, 20.0);
            r.hold_speed = constant_script<Real>(20.0);
            r.script.keys = {{0.0, four_wheel_input{0.0277, 0.0, 0.0, 0.0}}};
            r.steps = 8000;
            const four_wheel_state s = run(r);
            const four_wheel_forces f = evaluate(s, p, driver_input(r, s, r.steps * r.dt));
            Real F[2] = {0, 0}; // front, rear: the tyre forces along the body's left, laid flat
            for (std::size_t i = 0; i < kWheels; ++i)
            {
                const contact_geometry c = contact_of(p, s.body, i, f.steer, &s.corners[i]);
                const vec3 force = f.tyre[i].fx * c.forward + f.tyre[i].fy * c.left;
                F[is_front(i) ? 0 : 1] += dot(force, is_left(i) ? c.outboard : Real(-1) * c.outboard);
            }
            const Real kw = 50000 * mr * mr, kt = 250000, hz = s.body.pos.z;
            const Real Kf = kw * kt / (kw + kt) * p.track_front * p.track_front / 2;
            const Real Kr = kw * kt / (kw + kt) * p.track_rear * p.track_rear / 2;
            const Real Gf = kt / (kt + kw) * front * F[0], Gr = kt / (kt + kw) * rear * F[1];
            const Real a_y = std::hypot(s.body.vel.x, s.body.vel.y) * s.body.omega.z;
            const Real expected = (p.chassis.mass * a_y * hz - Gf - Gr) / (Kf + Kr - p.sprung_mass() * g * hz);
            const Real roll = roll_angle(s.body.q);
            const Real Mf = (f.fz[FR] - f.fz[FL]) / 2 * p.track_front, Mr = (f.fz[RR] - f.fz[RL]) / 2 * p.track_rear;
            const Real ef = Kf * expected + Gf, er = Kr * expected + Gr;
            std::printf("steady turn, roll centres %.2f %.2f, MR %.1f: roll %.6f (closed form %.6f), CG %.6f m, transfer "
                        "%.2f %.2f N m (K roll + G %.2f %.2f), share %.5f (closed form %.5f)\n",
                        front, rear, mr, roll, expected, hz, Mf, Mr, Kf * roll + Gf, Kr * roll + Gr, Mf / (Mf + Mr),
                        ef / (ef + er));
            check_near("steady turn: roll within 0.2% of the closed form", roll / expected, 1.0, 2e-3);
            check_near("steady turn: front transfer moment K roll + G", Mf, Kf * roll + Gf, 0.5);
            check_near("steady turn: rear transfer moment K roll + G", Mr, Kr * roll + Gr, 0.5);
            check_near("steady turn: front jacking difference is h F", (f.corner[FR].jacking - f.corner[FL].jacking) *
                       p.track_front / 2, front * F[0], 1e-6);
            check_near("steady turn: the loads add up to the weight", f.fz[FL] + f.fz[FR] + f.fz[RL] + f.fz[RR],
                       p.chassis.mass * g, 1e-6);
            return turn{roll, expected, Mf / (Mf + Mr), ef / (ef + er)};
        };
        const turn a = steady(0.10, 0.15, 1);
        check_near("steady turn: front share", a.share, a.expected_share, 1e-4);
        check_near("steady turn: front share value", a.share, 0.47636, 1e-4);
        const turn b = steady(0.25, 0, 1);
        check_near("steady turn, 0.25 and 0: front share", b.share, b.expected_share, 1e-4);
        check_near("steady turn, 0.25 and 0: front share value", b.share, 0.59560, 1e-4);
        steady(0.10, 0.15, 0.8);
    }

    // the toe-in run with roll centres 0.15 on both axles: each tyre's 120 N inboard gives slope x 120 = 24 N of
    // jacking, which the struts give back, so the body rises J / 50000. Catches jacking that does not reach the wheel.
    {
        four_wheel_params p = geometry_car(1, 0);
        p.set_tyre_curve(tyre_curve::linear);
        p.front_tyre.cornering_stiffness = p.rear_tyre.cornering_stiffness = 30000;
        p.lf = p.lr = 1.25;
        rebuild_tyres(p);
        p.front_corner.toe = p.rear_corner.toe = 0.004;
        p.front_corner.roll_centre = p.rear_corner.roll_centre = 0.15;
        four_wheel_run r;
        r.params = p;
        r.initial = four_wheel_initial(p, 20.0);
        r.hold_speed = constant_script<Real>(20.0);
        r.script.keys = {{0.0, four_wheel_input{0.0, 0.0, 0.0, 0.0}}};
        r.steps = 6000;
        const four_wheel_state s = run(r);
        const four_wheel_forces f = evaluate(s, p, driver_input(r, s, r.steps * r.dt));
        std::printf("toe-in, roll centres 0.15: jacking %.6f %.6f %.6f %.6f N, CG up %.9f m\n", f.corner[FL].jacking,
                    f.corner[FR].jacking, f.corner[RL].jacking, f.corner[RR].jacking, s.body.pos.z - p.cg_height);
        for (std::size_t i = 0; i < kWheels; ++i)
            check_near("toe, roll centres: jacking 24 N", f.corner[i].jacking, 24.0, 1e-3);
        check_near("toe, roll centres: the CG rises J / 50000", s.body.pos.z - p.cg_height, 0.00048, 1e-7);
    }

    // camber thrust, straight at 20 m/s held for 5 s, front camber -0.03 rad (tops inboard) and camber_thrust 0.1:
    // each front wheel leans in and pushes inboard, as a lateral slip of 0.003 the other way. Catches the same sign
    // on both sides (the car would drift).
    {
        four_wheel_params p = geometry_car(1, 0);
        p.front_corner.camber = -0.03;
        p.front_tyre.camber_thrust = 0.1;
        four_wheel_run r;
        r.params = p;
        r.initial = four_wheel_initial(p, 20.0);
        r.hold_speed = constant_script<Real>(20.0);
        r.script.keys = {{0.0, four_wheel_input{0.0, 0.0, 0.0, 0.0}}};
        r.steps = 5000;
        const four_wheel_state s = run(r);
        const four_wheel_forces f = evaluate(s, p, driver_input(r, s, r.steps * r.dt));
        const Real expected = tyre_forces(p.front_tyre, 0, 0.003, p.static_load(FL)).fy;
        std::printf("camber thrust: Fy FL %.4f N, FR %.4f N (expected %.4f), y %.3g m\n", f.tyre[FL].fy, f.tyre[FR].fy,
                    expected, s.body.pos.y);
        check_near("camber thrust: FL pushes right", f.tyre[FL].fy, -expected, 1e-3);
        check_near("camber thrust: FR pushes left", f.tyre[FR].fy, expected, 1e-3);
        check_near("camber thrust: the car has not moved sideways", s.body.pos.y, 0.0, 1e-9);
    }

    // steady left turn, 20 m/s held, linear tyres, camber gain -0.35 /m and thrust 0.1 on both axles. Each wheel's
    // camber to the road is its curve less (left) or plus (right) the body's roll, and each axle's force is
    // C x (the two steady lateral slips + the two camber slips). Catches a lean without the body's roll, a gain
    // that is ignored and a thrust that never reaches the wheel.
    {
        auto turn = [](Real thrust, four_wheel_params &p, four_wheel_run &r)
        {
            p = geometry_car(1, 0);
            p.set_tyre_curve(tyre_curve::linear);
            p.front_corner.camber_gain = p.rear_corner.camber_gain = -0.35;
            p.front_tyre.camber_thrust = p.rear_tyre.camber_thrust = thrust;
            r = four_wheel_run{};
            r.params = p;
            r.initial = four_wheel_initial(p, 20.0);
            r.hold_speed = constant_script<Real>(20.0);
            r.script.keys = {{0.0, four_wheel_input{0.0277, 0.0, 0.0, 0.0}}};
            r.steps = 8000;
            return run(r);
        };
        four_wheel_params p;
        four_wheel_run r;
        const four_wheel_state s = turn(0.1, p, r);
        const four_wheel_forces f = evaluate(s, p, driver_input(r, s, r.steps * r.dt));
        const Real roll = roll_angle(s.body.q);
        std::printf("camber turn: roll %.6f, camber FL %.5f FR %.5f\n", roll, f.camber[FL], f.camber[FR]);
        check_near("camber turn: FL is its curve less the roll", f.camber[FL], -0.35 * f.kin[FL].travel - roll, 1e-5);
        check_near("camber turn: FR is its curve plus the roll", f.camber[FR], -0.35 * f.kin[FR].travel + roll, 1e-5);
        for (std::size_t l : {FL, RL})
        {
            const std::size_t i = l + 1;
            const Real C = p.tyre(l).cornering_stiffness;
            Real slips = 0;
            for (std::size_t w : {l, i})
                slips += steady_lateral_slip(p.wheel, s.wheels[w].omega, f.patch[w]) +
                         (is_left(w) ? 1 : -1) * 0.1 * f.camber[w];
            const Real fy = f.tyre[l].fy + f.tyre[i].fy;
            std::printf("camber turn: %s axle Fy %.4f N, C x slips %.4f N\n", is_front(l) ? "front" : "rear", fy, C * slips);
            check_near(is_front(l) ? "camber turn: front axle force" : "camber turn: rear axle force", fy, C * slips, 0.01);
        }
        four_wheel_params p0;
        four_wheel_run r0;
        const four_wheel_state s0 = turn(0, p0, r0);
        const four_wheel_forces f0 = evaluate(s0, p0, driver_input(r0, s0, r0.steps * r0.dt));
        const Real with = f.tyre[FL].fy + f.tyre[FR].fy, without = f0.tyre[FL].fy + f0.tyre[FR].fy;
        std::printf("camber turn: front axle Fy %.4f N with thrust, %.4f N without\n", with, without);
        check_true("camber turn: the thrust changes the front axle force by more than 0.1 N",
                   std::fabs(with - without) > 0.1);
    }

    return finish("geometry");
}
