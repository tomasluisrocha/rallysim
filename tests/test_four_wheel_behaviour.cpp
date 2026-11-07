#include <algorithm>
#include <cmath>
#include <cstdio>
#include "scenarios/four_wheel_lane_change.hpp"
#include "scenarios/single_track_lane_change.hpp"
#include "tests/check.hpp"
#include "tests/single_track_helpers.hpp"

using namespace rallysim;

// The phase 4 gate (book chapter 15): driven symmetrically, the four-wheel car reproduces the single-track
// yaw rate within 5% in a 0.3 g steady turn, and its measured understeer gradient matches the formula within
// 10%. The single-track is the reference: same mass, geometry and loads, and each axle's two tyres together
// as stiff as the single-track's axle.

// The single-track car with the same numbers as a four-wheel car.
static single_track_params matching_single_track(const four_wheel_params &p)
{
    single_track_params q;
    q.chassis = p.chassis;
    q.lf = p.lf;
    q.lr = p.lr;
    q.front_cornering_stiffness = 2 * p.front_tyre.cornering_stiffness;
    q.rear_cornering_stiffness = 2 * p.rear_tyre.cornering_stiffness;
    q.friction_coefficient = p.front_tyre.friction_coefficient;
    q.load_sensitivity = p.front_tyre.load_sensitivity;
    q.nominal_wheel_load = p.front_tyre.nominal_load;
    q.cg_height = p.cg_height;
    q.track_width = p.track_front;
    q.front_transfer_share = p.front_transfer_share;
    q.max_steer = p.max_steer;
    return q;
}

// Four-wheel car at a held speed with a constant steer from the start.
static four_wheel_run four_wheel_constant_steer(const four_wheel_params &p, Real speed, Real steer, long steps)
{
    four_wheel_run r;
    r.params = p;
    r.initial = four_wheel_initial(p, speed);
    r.hold_speed = speed;
    r.script.keys = {{0.0, four_wheel_input{steer, 0.0}}};
    r.steps = steps;
    return r;
}

// Steer for a steady turn at lateral acceleration a_y and speed u, from the single-track's closed form.
static Real steer_for(const single_track_params &q, Real u, Real a_y)
{
    return q.wheelbase() * a_y / (u * u) + understeer_gradient(q) * a_y;
}

int main()
{
    const four_wheel_params p; // tyres 30 / 35 kN/rad, so the axles match the single-track's 60 / 70
    const single_track_params q = matching_single_track(p);

    // Gate 1: yaw rate within 5% of the single-track in a 0.3 g steady turn, at three speeds, with both vertical
    // models: the flat car (loads from the same formula as the single-track) and the car on springs (loads from
    // the springs; equal front and rear springs share the roll moment 50/50, as front_transfer_share does)
    for (vertical_model model : {vertical_model::planar, vertical_model::springs})
        for (Real speed : {10.0, 20.0, 30.0})
        {
            four_wheel_params pm = p;
            pm.vertical = model;
            const char *name = model == vertical_model::planar ? "planar" : "springs";
            const Real steer = steer_for(q, speed, 0.3 * kGravity);
            const four_wheel_state a = run(four_wheel_constant_steer(pm, speed, steer, 10000));
            const rigid_body_state b = run(constant_steer(q, speed, steer, 10000));
            const Real r4 = a.body.omega.z, r2 = b.omega.z;
            const Real a_y = std::hypot(velocity_in_body(a.body).u, velocity_in_body(a.body).v) * r4;
            std::printf("0.3 g at %2.0f m/s, %-7s: steer %.4f rad, yaw rate four-wheel %.5f, single-track %.5f (%+.2f%%), a_y %.2f m/s^2\n",
                        speed, name, steer, r4, r2, 100 * (r4 / r2 - 1), a_y);

            char what[160];
            std::snprintf(what, sizeof what, "gate (%s): yaw rate within 5%% of the single-track at 0.3 g, %.0f m/s", name, speed);
            check_true(what, std::fabs(r4 / r2 - 1) < 0.05);
            std::snprintf(what, sizeof what, "gate (%s): the turn is at about 0.3 g, %.0f m/s", name, speed);
            check_near(what, a_y / kGravity, 0.3, 0.03);
        }

    // Gate 2: the measured understeer gradient (steer - L/R) / a_y matches K = (Wf/Cf - Wr/Cr) / g within 10%,
    // for the default car and with the CG moved forward and back (equal tyres, so the CG alone sets the sign)
    {
        struct setup
        {
            const char *name;
            Real lf, lr, front_c, rear_c;
            int sign;
        };
        const setup setups[] = {{"default car", 1.3, 1.2, 30000, 35000, 1},
                                {"CG forward", 1.1, 1.4, 32500, 32500, 1},
                                {"CG rearward", 1.4, 1.1, 32500, 32500, -1}};
        for (const setup &c : setups)
        {
            four_wheel_params pc = p;
            pc.lf = c.lf;
            pc.lr = c.lr;
            pc.front_tyre.cornering_stiffness = c.front_c;
            pc.rear_tyre.cornering_stiffness = c.rear_c;
            const Real K = understeer_gradient(matching_single_track(pc));

            const Real speed = 20.0, steer = 0.02;
            const four_wheel_state s = run(four_wheel_constant_steer(pc, speed, steer, 10000));
            const body_velocity vel = velocity_in_body(s.body);
            const Real v_cg = std::hypot(vel.u, vel.v);
            const Real measured = (steer - pc.wheelbase() * vel.r / v_cg) / (v_cg * vel.r);
            std::printf("%s: K formula %.3e, measured %.3e (%+.1f%%)\n", c.name, K, measured, 100 * (measured / K - 1));

            char what[160];
            std::snprintf(what, sizeof what, "gate: %s, measured K within 10%% of the formula", c.name);
            check_true(what, std::fabs(measured / K - 1) < 0.10);
            std::snprintf(what, sizeof what, "gate: %s, K has the expected sign", c.name);
            check_true(what, c.sign > 0 ? measured > 0 : measured < 0);
        }
    }

    // The lane change, compared over the whole run: the four-wheel car follows the single-track's path and
    // yaw rate closely (it lags a little, because its tyres build force over a relaxation length)
    {
        four_wheel_run a = four_wheel_lane_change();
        single_track_run b = single_track_lane_change();
        b.params = q;
        four_wheel_state sa = a.initial;
        rigid_body_state sb = b.initial;
        Real gap = 0, worst_r = 0, peak_r = 0;
        for (long i = 0; i < a.steps; ++i)
        {
            step(sa, a.params, driver_input(a, sa, i * a.dt), a.dt);
            step(sb, b.params, driver_input(b, sb, i * b.dt), b.dt);
            gap = std::max(gap, std::hypot(sa.body.pos.x - sb.pos.x, sa.body.pos.y - sb.pos.y));
            worst_r = std::max(worst_r, std::fabs(sa.body.omega.z - sb.omega.z));
            peak_r = std::max(peak_r, std::fabs(sb.omega.z));
        }
        std::printf("lane change: largest position gap %.3f m, largest yaw-rate gap %.1f%% of the peak\n", gap, 100 * worst_r / peak_r);
        check_true("lane change: paths within 5 cm", gap < 0.05);
        check_true("lane change: yaw rate within 5% of its peak", worst_r < 0.05 * peak_r);
    }

    return finish("four_wheel_behaviour");
}
