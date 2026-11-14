#include <algorithm>
#include <cmath>
#include <cstdio>
#include "tests/check.hpp"
#include "vehicle/wheel.hpp"

using namespace rallysim;

// One wheel on its own, then on a straight-line rig: a quarter of the car (300 kg) riding on one wheel.
// The wheel: slip that cannot divide by zero, I domega/dt = T_drive - T_brake - Fx R, brakes that stop
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

    // steady slip ratio: a form that is finite everywhere
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

    // why the wheel is sub-stepped: the low-speed damping also damps the wheel's spin, at a rate c R^2 / I. Explicit
    // Euler is only stable while that rate times the step is below 2. At 30 kN s/m it is 2.5 per 1 ms step: each
    // step overshoots the last, so the spin flips sign every step. The damping force is capped at the tyre's grip,
    // which keeps the flipping bounded (about half a rad/s) instead of growing: the wheel chatters for ever. In
    // four sub-steps of 0.25 ms the rate is 0.6 per sub-step, and the same wheel settles.
    {
        struct chatter
        {
            Real late;  // largest spin in the last 0.1 s (rad/s)
            int flips;  // sign changes of the spin in that time
        };
        auto twisted_free_wheel = [&](int substeps)
        {
            rig r = rolling_rig(0.0);
            r.w.substeps = substeps;
            r.t.damping_x = 30000;
            r.s.slip_ratio = 0.02; // a twisted tyre on a free wheel at rest
            chatter c{0, 0};
            Real before = 0;
            for (int i = 0; i < 1000; ++i)
            {
                rig_step(r, 0.0, 0.0, dt);
                if (i >= 900)
                {
                    c.late = std::max(c.late, std::fabs(r.s.omega));
                    if (r.s.omega * before < 0)
                        ++c.flips;
                }
                before = r.s.omega;
            }
            return c;
        };
        const chatter one = twisted_free_wheel(1), four = twisted_free_wheel(4);
        std::printf("sub-stepping: damping 30 kN s/m, last 0.1 s: one 1 ms step %.3g rad/s, %d sign flips; "
                    "four 0.25 ms sub-steps %.3g rad/s, %d flips\n", one.late, one.flips, four.late, four.flips);
        check_true("sub-stepping: without it, strong damping makes the spin chatter (it flips sign nearly every step)",
                   one.flips > 50 && one.late > 0.1);
        check_true("sub-stepping: with four sub-steps the same wheel settles", four.flips == 0 && four.late < 1e-3);
    }

    // the limit on the damping: with the contact speed held at zero, the damping makes the spin update
    // omega' = omega (1 - k h), k h = damping_x R^2 h / I (weight is 1 at standstill). Explicit Euler: no overshoot
    // below k h = 1, a decaying ring up to 2, growth beyond. The test sets k h through damping_x on the real wheel,
    // one sub-step per step, with a spin so small that the grip cap stays out of the way. It catches a spin update
    // that loses stability below k h = 2, and a damping term whose size is not k = damping_x R^2 / I (the first
    // sub-step must leave 1 - k h of the spin, to the fade of weight at that speed).
    {
        struct spin
        {
            Real first, lowest, peak, last; // omega after one sub-step, its lowest, its largest |.| and its last, over omega0
        };
        const Real omega0 = 1e-6, h = 1e-3;
        auto spin_down = [&](Real kh)
        {
            wheel_params one = w;
            one.substeps = 1;
            tyre_params d = t;
            d.damping_x = kh * one.inertia / (one.radius * one.radius * h);
            d.damping_y = 0;
            wheel_state s;
            s.omega = omega0;
            spin out{0, 0, 0, 0};
            for (int i = 0; i < 400; ++i)
            {
                step_wheel(s, one, d, patch_velocity{0, 0}, 3000.0, 0.0, 0.0, h);
                if (i == 0)
                    out.first = s.omega / omega0;
                out.lowest = std::min(out.lowest, s.omega / omega0);
                out.peak = std::max(out.peak, std::fabs(s.omega) / omega0);
            }
            out.last = s.omega / omega0;
            return out;
        };
        const spin below_one = spin_down(0.95), above_one = spin_down(1.05), below_two = spin_down(1.9),
                   above_two = spin_down(2.1);
        std::printf("damping limit: k h 0.95: first %.3g, lowest %.3g, last %.3g; 1.05: first %.3g, lowest %.3g; "
                    "1.9: peak %.3g, last %.3g; 2.1: peak %.3g\n",
                    below_one.first, below_one.lowest, below_one.last, above_one.first, above_one.lowest,
                    below_two.peak, below_two.last, above_two.peak);
        check_near("damping limit: one sub-step leaves 1 - k h of the spin (k h 0.95)", below_one.first, 0.05, 1e-6);
        check_near("damping limit: one sub-step leaves 1 - k h of the spin (k h 1.9)", below_two.first, -0.9, 1e-6);
        // the relaxed slip is a weak spring on the spin, so the settled spin dips a fraction of a percent (0.2%) below
        // zero; an overshoot from the damping itself is 5% at k h 1.05
        check_true("damping limit: k h 0.95 settles without overshoot", below_one.lowest > -0.01 && std::fabs(below_one.last) < 1e-4);
        check_true("damping limit: k h 1.05 overshoots", above_one.lowest < -0.04 && above_one.first < 0);
        check_true("damping limit: k h 1.9 rings but settles, never above its start",
                   below_two.peak <= 1.0 + 1e-9 && std::fabs(below_two.last) < 1e-4);
        check_true("damping limit: k h 2.1 grows", above_two.peak > 1e3);

        // and the default wheel and tyre at the usual 1 ms step with its four sub-steps is well inside
        const Real kh = t.damping_x * w.radius * w.radius * (dt / w.substeps) / w.inertia;
        std::printf("damping limit: default wheel, dt 1 ms, %d sub-steps: k h = %.3f\n", w.substeps, kh);
        check_true("damping limit: the default wheel is below k h = 1", kh < 1.0);
    }

    // low speed, where the damping acts: a twisted tyre on a free wheel at rest unwinds and settles without the
    // spin update blowing up (the damping also acts on the spin), and so does a slow roll
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
