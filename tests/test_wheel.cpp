#include <algorithm>
#include <cmath>
#include <cstdio>
#include "tests/check.hpp"
#include "vehicle/wheel.hpp"

using namespace rallysim;

// One wheel on its own, then on a straight-line rig: a quarter of the car (300 kg) riding on one wheel.
// Book section 7.1: slip that cannot divide by zero, I domega/dt = T_drive - T_brake - Fx R, brakes that stop
// the wheel but never reverse it, and slip that builds over a relaxation length.

struct rig
{
    Real mass = 300; // kg carried by the wheel, and pushed by its tyre force
    wheel_params w;
    tyre_params t;
    wheel_state s;
    Real v = 0, x = 0; // forward speed (m/s) and distance (m)
};

static rig rolling_rig(Real speed)
{
    rig r;
    r.v = speed;
    r.s = rolling_wheel(r.w, speed);
    return r;
}

// One step: the wheel, then the mass with the force the wheel returned (semi-implicit, as the body).
static tyre_force rig_step(rig &r, Real drive, Real brake, Real dt)
{
    const tyre_force f = step_wheel(r.s, r.w, r.t, patch_velocity{r.v, 0}, r.mass * kGravity, drive, brake, dt);
    r.v += f.fx / r.mass * dt;
    r.x += r.v * dt;
    return f;
}

// Straight braking from `speed` with a brake strong enough to lock the wheel; distance to the first stop.
static Real braking_distance(Real speed, Real dt)
{
    rig r = rolling_rig(speed);
    while (r.v > 0)
        rig_step(r, 0.0, 3000.0, dt);
    return r.x;
}

int main()
{
    const wheel_params w; // R 0.315 m, I 1.2 kg m^2
    const tyre_params t;  // Cx 90 kN, C_alpha 30 kN/rad, sigma_x 0.2 m, sigma_y 0.4 m
    const Real dt = 0.001;

    // steady slip ratio: the book's form, finite everywhere
    {
        check_near("slip: rolling freely", steady_slip_ratio(w, 10.0 / w.radius, 10.0), 0.0, 1e-15);
        check_near("slip: locked wheel sliding forward is -1", steady_slip_ratio(w, 0.0, 10.0), -1.0);
        check_near("slip: wheel spinning at standstill is +1", steady_slip_ratio(w, 5.0 / w.radius, 0.0), 1.0);
        check_near("slip: driving, surface 10% faster", steady_slip_ratio(w, 11.0 / w.radius, 10.0), 1.0 / 11.0);
        check_near("slip: braking, surface 10% slower", steady_slip_ratio(w, 9.0 / w.radius, 10.0), -0.1);
        check_near("slip: at rest it is zero, not 0/0", steady_slip_ratio(w, 0.0, 0.0), 0.0);
        check_near("slip: reversing, rolling freely", steady_slip_ratio(w, -10.0 / w.radius, -10.0), 0.0, 1e-15);
        check_true("slip: reversing with the wheel spinning faster backwards pushes backwards",
                   steady_slip_ratio(w, -11.0 / w.radius, -10.0) < 0);
        check_near("lateral slip: moving right at 1 m/s at 10 m/s is +0.1 (wheel points left of its path)",
                   steady_lateral_slip(w, 10.0 / w.radius, patch_velocity{10.0, -1.0}), 0.1, 1e-15);
        check_near("lateral slip: at rest it is zero", steady_lateral_slip(w, 0.0, patch_velocity{0.0, 0.0}), 0.0);
    }

    // in the air (no load, no tyre force): omega = T t / I
    {
        wheel_state s;
        for (int i = 0; i < 1000; ++i)
            step_wheel(s, w, t, patch_velocity{}, 0.0, 120.0, 0.0, dt);
        check_near("air: omega = T t / I", s.omega, 120.0 * 1.0 / 1.2, 1e-9);
    }

    // brakes: they stop the wheel and hold it, and never spin it backwards
    {
        wheel_state s{50.0, 0, 0};
        bool never_negative = true;
        Real stopped_at = -1;
        for (int i = 1; i <= 1000; ++i)
        {
            step_wheel(s, w, t, patch_velocity{}, 0.0, 0.0, 300.0, dt);
            never_negative = never_negative && s.omega >= 0;
            if (stopped_at < 0 && s.omega == 0)
                stopped_at = i * dt;
        }
        check_true("brake: the wheel never turns backwards", never_negative);
        check_near("brake: stops at omega0 I / T = 0.2 s", stopped_at, 50.0 * 1.2 / 300.0, 1.5 * dt);
        check_true("brake: and stays exactly stopped", s.omega == 0.0);

        wheel_state back{-30.0, 0, 0};
        for (int i = 0; i < 1000; ++i)
            step_wheel(back, w, t, patch_velocity{}, 0.0, 0.0, 300.0, dt);
        check_true("brake: a wheel spinning backwards is stopped too, not sped up", back.omega == 0.0);

        wheel_state held;
        for (int i = 0; i < 1000; ++i)
            step_wheel(held, w, t, patch_velocity{}, 0.0, 100.0, 300.0, dt);
        check_true("brake: holds the wheel against a smaller drive torque", held.omega == 0.0);

        wheel_state over;
        for (int i = 0; i < 1000; ++i)
            step_wheel(over, w, t, patch_velocity{}, 0.0, 400.0, 300.0, dt);
        check_near("brake: a larger drive torque wins by the difference", over.omega, (400.0 - 300.0) / 1.2, 1e-9);
    }

    // relaxation: the slip builds over a rolling distance, 1 - exp(-d / sigma) after rolling d. Backward Euler
    // lags the exact curve by about half a step, so the tolerance is tied to the step: dt V / sigma. With no load
    // sensitivity and a big load, so the grip cap (tested below) stays out of the way.
    {
        tyre_params uncapped = t;
        uncapped.load_sensitivity = 0;
        const Real big_load = 1e5;
        struct result
        {
            Real got, exact, step; // fraction of the steady slip reached, exact fraction, dt V / sigma
        };
        auto relax = [&](Real speed, Real lengths, bool lateral, Real h)
        {
            wheel_state s = rolling_wheel(w, speed);
            const patch_velocity v{speed, lateral ? -0.05 * speed : 0.0};
            if (!lateral)
                s.omega = 1.05 * speed / w.radius; // surface 5% faster than the road, held
            const Real sigma = lateral ? t.relaxation_length_y : t.relaxation_length_x;
            const Real V = rolling_speed(w, s.omega, v.vx);
            const long steps = std::lround(lengths * sigma / V / h);
            for (long i = 0; i < steps; ++i)
                relax_slips(s, w, uncapped, v, big_load, h);
            const Real target = lateral ? steady_lateral_slip(w, s.omega, v) : steady_slip_ratio(w, s.omega, v.vx);
            const Real rolled = steps * h * V;
            return result{(lateral ? s.lateral_slip : s.slip_ratio) / target, 1 - std::exp(-rolled / sigma), h * V / sigma};
        };
        for (bool lateral : {true, false})
            for (Real speed : {2.0, 10.0, 30.0})
            {
                const result r = relax(speed, 1.0, lateral, dt);
                char what[128];
                std::snprintf(what, sizeof what, "relaxation: %s, 1 - exp(-1) after one length at %.0f m/s",
                              lateral ? "lateral" : "longitudinal", speed);
                check_near(what, r.got, r.exact, 0.25 * r.step);
            }
        const result coarse = relax(10.0, 1.0, false, 0.001), fine = relax(10.0, 1.0, false, 0.0005);
        check_true("relaxation: halving dt halves the lag", std::fabs(fine.got - fine.exact) < 0.6 * std::fabs(coarse.got - coarse.exact));
        check_near("relaxation: settled after ten lengths", relax(10.0, 10.0, true, dt).got, 1.0, 1e-4);
    }

    // standstill: a loaded wheel at rest with no input stays exactly at rest (no creep, no jitter)
    {
        rig r = rolling_rig(0.0);
        for (int i = 0; i < 10000; ++i)
            rig_step(r, 0.0, 0.0, dt);
        check_true("standstill: nothing moves in 10 s", r.v == 0.0 && r.x == 0.0 && r.s.omega == 0.0 &&
                                                            r.s.slip_ratio == 0.0 && r.s.lateral_slip == 0.0);
    }

    // rolling: with no torque the rig keeps its speed (there is no rolling resistance yet), at any speed
    for (Real speed : {0.5, 1.0, 5.0, 30.0})
    {
        rig r = rolling_rig(speed);
        for (int i = 0; i < 5000; ++i)
            rig_step(r, 0.0, 0.0, dt);
        char what[96];
        std::snprintf(what, sizeof what, "rolling at %.1f m/s: speed kept for 5 s", speed);
        check_near(what, r.v, speed, 1e-9);
    }

    // low speed, where the damping acts: a twisted tyre on a free wheel at rest unwinds and settles without the
    // spin update blowing up (the damping also acts on the spin, book chapter 14), and so does a slow roll
    {
        rig r = rolling_rig(0.0);
        r.s.slip_ratio = 0.02; // a twisted tyre, about 4 mm, on a free wheel
        Real worst_omega = 0;
        for (int i = 0; i < 2000; ++i)
        {
            rig_step(r, 0.0, 0.0, dt);
            worst_omega = std::max(worst_omega, std::fabs(r.s.omega));
        }
        check_true("low speed: a twisted free wheel stays finite (no blow-up)", std::isfinite(r.s.omega) && worst_omega < 10.0);
        check_true("low speed: and settles", std::fabs(r.v) < 1e-3 && std::fabs(r.s.omega) < 1e-2);

        rig slow = rolling_rig(0.1);
        for (int i = 0; i < 5000; ++i)
            rig_step(slow, 0.0, 0.0, dt);
        check_near("low speed: rolling at 0.1 m/s keeps its speed (the damping only acts on sliding)", slow.v, 0.1, 1e-9);
    }

    // gentle drive: no wheelspin, a = (T / R) / (m + I / R^2): the wheel's spin inertia adds to the mass
    {
        rig r = rolling_rig(10.0);
        const Real T = 300.0;
        for (int i = 0; i < 2000; ++i)
            rig_step(r, T, 0.0, dt);
        const Real expected = (T / w.radius) / (r.mass + w.inertia / (w.radius * w.radius));
        check_near("drive: a = (T/R) / (m + I/R^2)", (r.v - 10.0) / 2.0, expected, 0.01 * expected);
        check_true("drive: small slip (no wheelspin)", r.s.slip_ratio > 0 && r.s.slip_ratio < 0.02);
    }

    // hard drive: the wheel spins up and the rig accelerates at the friction limit, mu(Fz) g
    {
        rig r = rolling_rig(10.0);
        for (int i = 0; i < 2000; ++i)
            rig_step(r, 1500.0, 0.0, dt);
        const Real mu = peak_friction(t, r.mass * kGravity);
        check_near("wheelspin: a = mu g", (r.v - 10.0) / 2.0, mu * kGravity, 0.02 * mu * kGravity);
        check_true("wheelspin: the wheel surface runs well ahead of the road", r.s.omega * w.radius > 1.2 * r.v);
    }

    // straight braking with a locked wheel: stops in v^2 / (2 mu g), and the wheel never turns backwards
    {
        rig r = rolling_rig(20.0);
        bool never_backwards = true;
        while (r.v > 0)
        {
            rig_step(r, 0.0, 3000.0, dt);
            never_backwards = never_backwards && r.s.omega >= 0;
        }
        const Real mu = peak_friction(t, r.mass * kGravity);
        const Real expected = 20.0 * 20.0 / (2 * mu * kGravity);
        std::printf("locked-wheel stop from 20 m/s: %.3f m (v^2 / 2 mu g = %.3f m)\n", r.x, expected);
        check_near("braking: distance = v^2 / (2 mu g) within 2%", r.x, expected, 0.02 * expected);
        check_true("braking: the wheel is locked, not turning backwards", never_backwards && r.s.omega == 0.0);

        // while sliding, the twist is capped at what the grip can hold: grip / Cx, about 3%, not -1
        const Real cap = tyre_grip(t, r.mass * kGravity) / t.longitudinal_stiffness;
        check_true("cap: a locked tyre is twisted only as far as its grip holds", std::fabs(r.s.slip_ratio) <= cap * (1 + 1e-12));

        // after the stop: the brake holds the wheel and the rig settles within half a second, without rolling back
        const Real stopped_at = r.x;
        Real settle = 0, after = 0, back = 0;
        for (int i = 0; i < 3000; ++i)
        {
            rig_step(r, 0.0, 3000.0, dt);
            (i < 500 ? settle : after) = std::max(i < 500 ? settle : after, std::fabs(r.v));
            back = std::max(back, stopped_at - r.x);
        }
        std::printf("after the stop: largest |v| in the first 0.5 s %.4f m/s, afterwards %.6f m/s, rolled back %.4f m\n",
                    settle, after, back);
        check_true("after the stop: below 1 cm/s after half a second, and it stays there", after < 0.01);
        // the tyre is still twisted by sigma_x * cap when it stops sliding; it unwinds, so the rig moves back by
        // about that much (6.7 mm here), plus a little overshoot
        const Real twist = t.relaxation_length_x * cap;
        check_true("after the stop: it rolls back less than twice the tyre's capped twist", back < 2 * twist);
        check_true("after the stop: the wheel stays locked", r.s.omega == 0.0);

        // halving dt converges
        const Real d1 = braking_distance(20.0, 0.002), d2 = braking_distance(20.0, 0.001), d3 = braking_distance(20.0, 0.0005);
        check_true("braking: halving dt shrinks the change", std::fabs(d3 - d2) < std::fabs(d2 - d1));
    }

    return finish("wheel");
}
