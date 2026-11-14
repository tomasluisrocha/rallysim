#include <cmath>
#include <cstdio>
#include <vector>
#include "tests/check.hpp"
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

    return finish("geometry");
}
