#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
#include "scenarios/run_single_track.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// Whole-car behaviour of the single-track model against closed forms: understeer gradient with the
// centre of gravity moved, the steering-step response, and cornering at the friction limit.
// The functions of the model are tested one by one in test_single_track.cpp.

// Steady-state understeer gradient of the car, rad per m/s^2 of lateral acceleration.
static Real understeer_gradient(const single_track_params &p)
{
    const axle_pair w = axle_loads(p);
    return (w.front / p.front_cornering_stiffness - w.rear / p.rear_cornering_stiffness) / kGravity;
}

// A run at constant speed (held) and constant steer from the start.
static single_track_run constant_steer(const single_track_params &p, Real speed, Real steer, long steps)
{
    single_track_run r;
    r.params = p;
    r.initial = single_track_initial(speed);
    r.hold_speed = speed;
    r.script.keys = {{0.0, single_track_input{steer, 0.0}}};
    r.steps = steps;
    return r;
}

// Measured understeer gradient from a settled circle: (steer - L / R) / a_y.
static Real measured_gradient(const single_track_params &p, Real speed, Real steer)
{
    const rigid_body_state s = run(constant_steer(p, speed, steer, 10000));
    const body_velocity vel = velocity_in_body(s);
    const Real v_cg = std::hypot(vel.u, vel.v);
    return (steer - p.wheelbase() / (v_cg / vel.r)) / (v_cg * vel.r);
}

// Yaw rate against time (one value per step, starting at 0) after a steer step of `steer` at `speed`.
static std::vector<Real> simulated_yaw_rate(const single_track_params &p, Real speed, Real steer, long steps)
{
    const single_track_run r = constant_steer(p, speed, steer, steps);
    rigid_body_state s = r.initial;
    std::vector<Real> out{velocity_in_body(s).r};
    for (long i = 0; i < steps; ++i)
    {
        step(s, p, driver_input(r, s, i * r.dt), r.dt);
        out.push_back(velocity_in_body(s).r);
    }
    return out;
}

// The same response from the linearised equations in (v, r) at constant forward speed u:
//   v' = -(Cf + Cr) / (m u) v - (u + (Cf lf - Cr lr) / (m u)) r + Cf / m steer
//   r' = -(Cf lf - Cr lr) / (Iz u) v - (Cf lf^2 + Cr lr^2) / (Iz u) r + Cf lf / Iz steer
// integrated with fourth-order Runge-Kutta, which is exact for this purpose at a 1 ms step. It
// shares no code with the simulation, so it is an independent oracle for small steer.
static std::vector<Real> linear_yaw_rate(const single_track_params &p, Real u, Real steer, Real dt, long steps)
{
    const Real m = p.chassis.mass, iz = p.chassis.inertia.z;
    const Real cf = p.front_cornering_stiffness, cr = p.rear_cornering_stiffness;
    const Real a11 = -(cf + cr) / (m * u);
    const Real a12 = -u - (cf * p.lf - cr * p.lr) / (m * u);
    const Real a21 = -(cf * p.lf - cr * p.lr) / (iz * u);
    const Real a22 = -(cf * p.lf * p.lf + cr * p.lr * p.lr) / (iz * u);
    const Real b1 = cf / m * steer, b2 = cf * p.lf / iz * steer;

    auto derivative = [&](Real v, Real r, Real &dv, Real &dr)
    {
        dv = a11 * v + a12 * r + b1;
        dr = a21 * v + a22 * r + b2;
    };

    Real v = 0, r = 0;
    std::vector<Real> out{r};
    for (long i = 0; i < steps; ++i)
    {
        Real k1v, k1r, k2v, k2r, k3v, k3r, k4v, k4r;
        derivative(v, r, k1v, k1r);
        derivative(v + 0.5 * dt * k1v, r + 0.5 * dt * k1r, k2v, k2r);
        derivative(v + 0.5 * dt * k2v, r + 0.5 * dt * k2r, k3v, k3r);
        derivative(v + dt * k3v, r + dt * k3r, k4v, k4r);
        v += dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v);
        r += dt / 6 * (k1r + 2 * k2r + 2 * k3r + k4r);
        out.push_back(r);
    }
    return out;
}

// What the book asks of a steering step: overshoot, peak time and settling time of the yaw rate.
struct step_response
{
    Real final = 0;     // value at the end of the record
    Real overshoot = 0; // peak / final - 1
    Real t_peak = 0;    // s
    Real t_settle = 0;  // s, after this the yaw rate stays within 5% of the final value
};

static step_response measure(const std::vector<Real> &r, Real dt)
{
    step_response m;
    m.final = r.back();
    std::size_t peak = 0, last_outside = 0;
    for (std::size_t i = 0; i < r.size(); ++i)
    {
        if (r[i] > r[peak])
            peak = i;
        if (std::fabs(r[i] - m.final) > 0.05 * std::fabs(m.final))
            last_outside = i;
    }
    m.overshoot = r[peak] / m.final - 1.0;
    m.t_peak = static_cast<Real>(peak) * dt;
    m.t_settle = static_cast<Real>(last_outside) * dt;
    return m;
}

int main()
{
    const single_track_params p; // 1200 kg, lf = 1.3, lr = 1.2, Cf = 60000, Cr = 70000, mu = 1
    const Real dt = 0.001;

    // Gate: the measured understeer gradient is within a few percent of the formula, and moving the
    // centre of gravity flips the balance. Equal stiffness on both axles, so only the weight
    // distribution decides: a forward CG loads the front (understeer), a rearward CG the rear (oversteer).
    {
        struct setup
        {
            const char *name;
            Real lf, lr;
            int sign; // expected sign of K
        };
        const setup setups[] = {{"CG forward", 1.1, 1.4, 1},
                                {"CG centred", 1.25, 1.25, 0},
                                {"CG rearward", 1.4, 1.1, -1}};

        for (const setup &c : setups)
            for (Real speed : {10.0, 20.0})
            {
                single_track_params q = p;
                q.lf = c.lf;
                q.lr = c.lr;
                q.front_cornering_stiffness = q.rear_cornering_stiffness = 65000;
                const Real K = understeer_gradient(q);
                const Real measured = measured_gradient(q, speed, 0.02);

                char what[160];
                std::snprintf(what, sizeof what, "%s at %.0f m/s: measured K has the expected sign", c.name, speed);
                if (c.sign > 0)
                    check_true(what, K > 0 && measured > 0);
                else if (c.sign < 0)
                    check_true(what, K < 0 && measured < 0);

                std::snprintf(what, sizeof what, "%s at %.0f m/s: measured K within 3%% of the formula", c.name, speed);
                if (c.sign != 0)
                    check_true(what, std::fabs(measured / K - 1.0) < 0.03);
                else
                    check_near(what, measured, 0.0, 1e-5); // neutral: no understeer to measure
            }

        // the default car as well, at two speeds
        for (Real speed : {10.0, 20.0})
        {
            const Real K = understeer_gradient(p);
            const Real measured = measured_gradient(p, speed, 0.03);
            char what[96];
            std::snprintf(what, sizeof what, "default car at %.0f m/s: measured K within 3%% of the formula", speed);
            check_true(what, K > 0 && std::fabs(measured / K - 1.0) < 0.03);
        }
    }

    // Steering step: a small step (0.005 rad, so the tyres stay linear) is compared with the
    // linearised model, and its overshoot, peak time and settling time with the same oracle. At
    // 20 m/s the car is close to critically damped (damping ratio about 0.96); at 50 m/s it is
    // clearly underdamped (about 0.78), so the overshoot is a real test.
    {
        const Real steer = 0.005;
        const long steps = 8000;

        for (Real speed : {20.0, 50.0})
        {
            const std::vector<Real> sim = simulated_yaw_rate(p, speed, steer, steps);
            const std::vector<Real> oracle = linear_yaw_rate(p, speed, steer, dt, steps);
            const step_response ms = measure(sim, dt);
            const step_response mo = measure(oracle, dt);

            char what[160];
            Real worst = 0;
            for (std::size_t i = 0; i < sim.size(); ++i)
                worst = std::max(worst, std::fabs(sim[i] - oracle[i]));
            std::snprintf(what, sizeof what, "step at %.0f m/s: the yaw rate follows the linear model within 1%% of its final value", speed);
            check_true(what, worst < 0.01 * std::fabs(mo.final));

            // the settled yaw rate has a closed form: r = steer / (L / u + K u)
            const Real closed = steer / (p.wheelbase() / speed + understeer_gradient(p) * speed);
            std::snprintf(what, sizeof what, "step at %.0f m/s: settled yaw rate = steer / (L/u + K u), within 0.2%%", speed);
            check_true(what, std::fabs(ms.final / closed - 1.0) < 0.002);

            std::snprintf(what, sizeof what, "step at %.0f m/s: overshoot matches the linear model", speed);
            check_near(what, ms.overshoot, mo.overshoot, 0.01);

            std::snprintf(what, sizeof what, "step at %.0f m/s: settling time matches the linear model", speed);
            check_near(what, ms.t_settle, mo.t_settle, 0.05);

            if (speed < 30)
            {
                std::snprintf(what, sizeof what, "step at %.0f m/s: close to critical damping, overshoot below 1%%", speed);
                check_true(what, ms.overshoot >= 0 && ms.overshoot < 0.01);
            }
            else
            {
                std::snprintf(what, sizeof what, "step at %.0f m/s: underdamped, the yaw rate overshoots by more than 10%%", speed);
                check_true(what, ms.overshoot > 0.10);
                std::snprintf(what, sizeof what, "step at %.0f m/s: peak time matches the linear model", speed);
                check_near(what, ms.t_peak, mo.t_peak, 0.03);
            }
        }
    }

    // Friction-limited cornering: far more steer than the grip can follow, at a held speed. The
    // sideways force on the body never exceeds mu m g, and the front axle ends at its limit. With
    // the front axle saturated, the yaw balance gives the rear axle lf / lr times the front force
    // cos(steer), so the total sideways force is Fyf cos(steer) L / lr and the settled lateral
    // acceleration is
    //   a_y = mu cos(steer) (g - a_x h / lr)
    // The speed controller has to push forward to cancel the pull of the steered front tyre, and that
    // acceleration a_x moves load to the rear, which lowers the front limit: load transfer at work.
    {
        struct setup
        {
            Real mu, steer;
        };
        for (const setup &c : {setup{1.0, 0.2}, setup{1.0, 0.3}, setup{0.6, 0.3}})
        {
            single_track_params q = p;
            q.friction_coefficient = c.mu;
            q.load_sensitivity = 0; // the closed form below has grip = mu Fz; sensitivity is tested after this block
            const single_track_run r = constant_steer(q, 15.0, c.steer, 20000);

            const Real limit = c.mu * q.chassis.mass * kGravity;
            Real worst_force = 0;
            rigid_body_state s = r.initial;
            for (long i = 0; i < r.steps; ++i)
            {
                const single_track_input in = driver_input(r, s, i * r.dt);
                worst_force = std::max(worst_force, std::fabs(to_body(s.q, single_track_wrench(s, q, in).force).y));
                step(s, q, in, r.dt);
            }

            char what[160];
            std::snprintf(what, sizeof what, "mu %.1f, steer %.1f: the sideways force never exceeds mu m g", c.mu, c.steer);
            check_true(what, worst_force <= limit + 1e-6);

            const single_track_input in = driver_input(r, s, r.steps * r.dt);
            const single_track_forces f = evaluate(s, q, in);
            const Real a_x = f.a_x;
            const body_velocity vel = f.vel;
            const axle_pair loads = axle_loads(q, a_x);
            std::snprintf(what, sizeof what, "mu %.1f, steer %.1f: the front axle is at its friction limit", c.mu, c.steer);
            check_near(what, f.fy.front, c.mu * loads.front, 1e-6);

            std::snprintf(what, sizeof what, "mu %.1f, steer %.1f: the speed controller pushes forward (load moves to the rear)", c.mu, c.steer);
            check_true(what, a_x > 0.5 && loads.front < axle_loads(q).front);

            const Real a_y = std::hypot(vel.u, vel.v) * vel.r;
            const Real expected = c.mu * std::cos(c.steer) * (kGravity - a_x * q.cg_height / q.lr);
            std::snprintf(what, sizeof what, "mu %.1f, steer %.1f: settled lateral acceleration = mu cos(steer) (g - a_x h / lr), within 1%%", c.mu, c.steer);
            check_true(what, std::fabs(a_y / expected - 1.0) < 0.01);
        }
    }

    // Load sensitivity at the limit: lateral load transfer lowers the grip of the axle that carries it,
    // so it cuts the cornering limit, and moving the transfer to the front (front_transfer_share up, like a
    // stiffer front anti-roll bar) makes a front-limited car reach a lower lateral acceleration. Below the
    // limit nothing changes, because the cornering stiffness does not depend on the load.
    {
        auto settled_a_y = [&](Real sensitivity, Real share, Real steer)
        {
            single_track_params q = p;
            q.load_sensitivity = sensitivity;
            q.front_transfer_share = share;
            const rigid_body_state s = run(constant_steer(q, 15.0, steer, 20000));
            const body_velocity vel = velocity_in_body(s);
            return std::hypot(vel.u, vel.v) * vel.r;
        };

        const Real off = settled_a_y(0.0, 0.5, 0.3), on = settled_a_y(0.2, 0.5, 0.3);
        check_true("sensitivity: load transfer lowers the cornering limit", on < off - 0.05);

        // 0.5 and 0.7 keep this car front-limited; at 0.3 the rear loses enough grip to become the limit
        const Real front_share = settled_a_y(0.2, 0.7, 0.3), even_share = settled_a_y(0.2, 0.5, 0.3);
        check_true("sensitivity: more transfer on the front axle, lower limit for a front-limited car",
                   front_share < even_share - 0.05);

        check_near("sensitivity: no effect in the linear range", settled_a_y(0.2, 0.7, 0.02), settled_a_y(0.0, 0.5, 0.02), 1e-9);
    }

    return finish("single_track_behaviour");
}