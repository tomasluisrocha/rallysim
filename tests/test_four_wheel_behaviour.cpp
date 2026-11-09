#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
#include "scenarios/four_wheel_lane_change.hpp"
#include "scenarios/single_track_lane_change.hpp"
#include "tests/check.hpp"
#include "tests/single_track_helpers.hpp"

using namespace rallysim;

// The phase 4 gate: driven symmetrically, the four-wheel car reproduces the single-track
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
    // The car's own tyre is tmeasy (phase 5). The closed forms below (the understeer gradient, the linear step
    // response, the friction limit with a flat ceiling) are for the linear tyre, so those checks run the same car on
    // it; the 0.3 g turn and the lane change run on the car's own tyre (the phase 3 tests,
    // rerun on four wheels with the new tyre).
    four_wheel_params linear = p;
    linear.set_tyre_curve(tyre_curve::linear);

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
            four_wheel_params pc = linear;
            pc.lf = c.lf;
            pc.lr = c.lr;
            pc.front_tyre.cornering_stiffness = c.front_c;
            pc.rear_tyre.cornering_stiffness = c.rear_c;
            const Real K = understeer_gradient(matching_single_track(pc));

            const Real speed = 20.0, steer = 0.02;
            auto measure_K = [&](const four_wheel_params &car)
            {
                const four_wheel_state s = run(four_wheel_constant_steer(car, speed, steer, 10000));
                const body_velocity vel = velocity_in_body(s.body);
                const Real v_cg = std::hypot(vel.u, vel.v);
                return (steer - car.wheelbase() * vel.r / v_cg) / (v_cg * vel.r);
            };
            const Real measured = measure_K(pc);
            four_wheel_params peak = pc;
            peak.set_tyre_curve(tyre_curve::peak_slide);
            const Real measured_peak = measure_K(peak);
            std::printf("%s: K formula %.3e, measured %.3e (%+.1f%%); on peak_slide %.3e (%+.1f%%)\n", c.name, K,
                        measured, 100 * (measured / K - 1), measured_peak, 100 * (measured_peak / K - 1));

            char what[160];
            std::snprintf(what, sizeof what, "gate: %s, measured K within 10%% of the formula", c.name);
            check_true(what, std::fabs(measured / K - 1) < 0.10);
            std::snprintf(what, sizeof what, "gate: %s, K has the expected sign", c.name);
            check_true(what, c.sign > 0 ? measured > 0 : measured < 0);
            // On peak_slide the cornering stiffness falls as the slip grows (f(s) / s = 1 - s / 4 near zero for the
            // default peak at 2), and the axle that works harder loses more: the understeering car understeers more,
            // the oversteering car oversteers more. So |K| grows as the car corners harder.
            std::snprintf(what, sizeof what, "peak_slide: %s, |K| larger than the linear formula's, by under 15%%", c.name);
            check_true(what, measured_peak / K > 1.0 && measured_peak / K < 1.15);
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

    // Steering step on four wheels (overshoot and settling as the linear model predicts, within
    // 10%), on springs, on the linear tyre (on peak_slide the stiffness already drops a few percent). A small step (0.005 rad, so the tyres stay linear) from the start, at 20 m/s (close to
    // critically damped) and 50 m/s (clearly underdamped). The four-wheel car adds what the linear model leaves out:
    // tyre relaxation, body roll and the inner and outer wheels' different slip angles.
    {
        const Real steer = 0.005, dt = 0.001;
        const long steps = 8000;
        for (Real speed : {20.0, 50.0})
        {
            const four_wheel_run r = four_wheel_constant_steer(linear, speed, steer, steps);
            four_wheel_state s = r.initial;
            std::vector<Real> sim{s.body.omega.z};
            for (long i = 0; i < steps; ++i)
            {
                step(s, r.params, driver_input(r, s, i * r.dt), r.dt);
                sim.push_back(s.body.omega.z);
            }
            const step_response ms = measure(sim, dt);
            const step_response mo = measure(linear_yaw_rate(q, speed, steer, dt, steps), dt);
            const Real closed = steer / (q.wheelbase() / speed + understeer_gradient(q) * speed);
            std::printf("step at %.0f m/s: settled yaw rate %.5f (closed form %.5f); peak %.5f at %.3f s (linear %.5f at %.3f s); "
                        "settled by %.3f s (linear %.3f s)\n", speed, ms.final, closed, ms.final * (1 + ms.overshoot), ms.t_peak,
                        mo.final * (1 + mo.overshoot), mo.t_peak, ms.t_settle, mo.t_settle);

            char what[160];
            std::snprintf(what, sizeof what, "step at %.0f m/s: settled yaw rate = steer / (L/u + K u), within 2%%", speed);
            check_true(what, std::fabs(ms.final / closed - 1.0) < 0.02);
            std::snprintf(what, sizeof what, "step at %.0f m/s: peak yaw rate within 10%% of the linear model's", speed);
            check_true(what, std::fabs((1 + ms.overshoot) * ms.final / ((1 + mo.overshoot) * mo.final) - 1.0) < 0.10);
            std::snprintf(what, sizeof what, "step at %.0f m/s: settling time within 10%% of the linear model's", speed);
            check_true(what, std::fabs(ms.t_settle / mo.t_settle - 1.0) < 0.10);
            if (speed > 30)
            {
                std::snprintf(what, sizeof what, "step at %.0f m/s: underdamped, it overshoots by more than 10%% as the linear model does", speed);
                check_true(what, ms.overshoot > 0.10 && mo.overshoot > 0.10);
            }
        }
    }

    // Cornering at the friction limit on four wheels: far more steer than the grip can follow,
    // at a held speed, with load sensitivity off so each tyre's grip is mu Fz. Both front tyres saturate, at
    // mu Fz_FL and mu Fz_FR across their wheels. The rear supplies what the yaw balance needs:
    //   lr F_rear = lf (F_FL + F_FR) cos(steer) + (t / 2) sin(steer) (F_FL - F_FR)
    // The last term is what four wheels add to the single-track's version: each steered front tyre's force leans
    // back by the steer angle, so it also drags the car, and the two drags act at +-t/2 from the centreline. The
    // outer front carries more load, drags more, and turns the car out of the corner. So the settled lateral
    // acceleration is
    //   a_y = mu [ (Fz_FL + Fz_FR) cos(steer) L / lr + (t / 2) sin(steer) (Fz_FL - Fz_FR) / lr ] / m
    // from the front loads (the springs tests check those). The inner rear may saturate too (with an open
    // differential it gets the same drive torque as the outer, on less load); the outer rear keeps some grip.
    //
    // The steer is ramped in over 2 s, and mu = 1 is run only at 0.2 rad: stepped in, or at mu = 1 with 0.3 rad (so
    // much front drag that the drive needed saturates both rear tyres), this rear-drive car spins, the speed
    // controller's full throttle taking the rear's side grip (power oversteer). Real, but not the front-limited state
    // this test is about.
    {
        struct setup
        {
            Real mu, steer;
        };
        for (const setup &c : {setup{1.0, 0.2}, setup{0.6, 0.2}, setup{0.6, 0.3}})
        {
            four_wheel_params pm = linear; // the closed form needs a tyre that holds mu Fz while it slides
            for (tyre_params *t : {&pm.front_tyre, &pm.rear_tyre})
            {
                t->friction_coefficient = c.mu;
                t->load_sensitivity = 0;
            }
            four_wheel_run r = four_wheel_constant_steer(pm, 15.0, c.steer, 20000);
            r.script.keys = {{0.0, four_wheel_input{}}, {2.0, four_wheel_input{c.steer, 0.0, 0.0, 0.0}},
                             {20.0, four_wheel_input{c.steer, 0.0, 0.0, 0.0}}};
            const four_wheel_state s = run(r);
            const four_wheel_forces f = evaluate(s, pm, driver_input(r, s, r.steps * r.dt));
            const body_velocity v = velocity_in_body(s.body);
            const Real a_y = std::hypot(v.u, v.v) * v.r;
            const Real L = pm.wheelbase(), t = pm.track_front;
            const Real expected = c.mu * ((f.fz[FL] + f.fz[FR]) * std::cos(c.steer) * L / pm.lr +
                                          0.5 * t * std::sin(c.steer) * (f.fz[FL] - f.fz[FR]) / pm.lr) / pm.chassis.mass;
            const Real centreline = c.mu * (f.fz[FL] + f.fz[FR]) * std::cos(c.steer) * L / pm.lr / pm.chassis.mass;
            auto use = [&](std::size_t w)
            { return std::hypot(f.tyre[w].fx, f.tyre[w].fy) / (c.mu * f.fz[w]); };
            std::printf("limit, mu %.1f, steer %.1f: a_y %.3f m/s^2 (four-wheel form %.3f; without the drag term %.3f); grip use FL %.3f FR %.3f RL %.3f RR %.3f\n",
                        c.mu, c.steer, a_y, expected, centreline, use(FL), use(FR), use(RL), use(RR));

            char what[160];
            std::snprintf(what, sizeof what, "limit, mu %.1f, steer %.1f: both front tyres at their grip, the outer rear below its own", c.mu, c.steer);
            check_true(what, use(FL) > 0.999 && use(FR) > 0.999 && use(RR) < 0.999);
            std::snprintf(what, sizeof what, "limit, mu %.1f, steer %.1f: settled a_y = the four-wheel friction-limit form, within 1%%", c.mu, c.steer);
            check_true(what, std::fabs(a_y / expected - 1.0) < 0.01);
            std::snprintf(what, sizeof what, "limit, mu %.1f, steer %.1f: speed held (not spinning)", c.mu, c.steer);
            check_near(what, v.u, 15.0, 0.5);
        }
    }

    return finish("four_wheel_behaviour");
}
