#include <algorithm>
#include <cmath>
#include <cstdio>
#include "scenarios/four_wheel_driving.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// Phase 4, step 4: pedals, drive layouts, brakes and handbrake. The straight-line tests use load sensitivity off,
// so the closed forms (book section 11.2: braking v^2 / 2 mu g, acceleration at the traction limit) are exact.

static four_wheel_params flat_grip(Real split_front = 0)
{
    four_wheel_params p;
    p.front_tyre.load_sensitivity = 0;
    p.rear_tyre.load_sensitivity = 0;
    p.drive_brake.drive_split_front = split_front;
    return p;
}

// Average acceleration over 0.6 s of full throttle from 4 m/s, after 0.2 s to let the wheels spin up. Low
// speed, so the engine still has the torque to reach the traction limit (the power limit is tested apart).
static Real full_throttle_acceleration(const four_wheel_params &p)
{
    four_wheel_state s = four_wheel_initial(p, 4.0);
    const four_wheel_input in{0.0, 1.0, 0.0, 0.0};
    const Real dt = 0.001;
    for (int i = 0; i < 200; ++i)
        step(s, p, in, dt);
    const Real u0 = velocity_in_body(s.body).u;
    for (int i = 0; i < 600; ++i)
        step(s, p, in, dt);
    return (velocity_in_body(s.body).u - u0) / 0.6;
}

int main()
{
    const Real dt = 0.001;

    // pedals to torques: the split, the bias, the handbrake on the rears only, pedals clamped to 0..1 (wheels at
    // rest, so the power limit does not come in)
    const std::array<Real, 4> still{};
    {
        drive_brake_params d; // 6000 drive, 6000 brake (70% front), 3000 handbrake
        auto t = wheel_torques(d, pedals{1.0, 0.0, 0.0}, still);
        check_true("RWD: all the drive on the rear wheels, half each", t[FL].drive == 0 && t[FR].drive == 0 && t[RL].drive == 3000 && t[RR].drive == 3000);
        d.drive_split_front = 1;
        t = wheel_torques(d, pedals{0.5, 0.0, 0.0}, still);
        check_true("FWD: half throttle, 1500 N m on each front wheel", t[FL].drive == 1500 && t[FR].drive == 1500 && t[RL].drive == 0);
        d.drive_split_front = 0.5;
        t = wheel_torques(d, pedals{1.0, 0.0, 0.0}, still);
        check_true("AWD: 1500 N m on every wheel", t[FL].drive == 1500 && t[RR].drive == 1500);

        t = wheel_torques(d, pedals{0.0, 1.0, 0.0}, still);
        check_near("brake: front wheel gets 70% / 2", t[FL].brake, 0.70 * 6000 / 2, 1e-9);
        check_near("brake: rear wheel gets 30% / 2", t[RL].brake, 0.30 * 6000 / 2, 1e-9);
        check_near("brake: the four add up to the full brake torque", t[FL].brake + t[FR].brake + t[RL].brake + t[RR].brake, 6000.0, 1e-9);

        t = wheel_torques(d, pedals{0.0, 0.0, 1.0}, still);
        check_true("handbrake: rear wheels only, half each", t[FL].brake == 0 && t[FR].brake == 0 && t[RL].brake == 1500 && t[RR].brake == 1500);

        const auto over = wheel_torques(d, pedals{2.0, -1.0, 5.0}, still);
        const auto full = wheel_torques(d, pedals{1.0, 0.0, 1.0}, still);
        check_true("pedals are clamped to 0..1", over[RL].drive == full[RL].drive && over[FL].brake == 0 && over[RL].brake == full[RL].brake);
    }

    // power limit: full torque at low wheel speed, P / omega above P / T_max (200 kW / 6000 N m = 33.3 rad/s)
    {
        const drive_brake_params d;
        check_near("power: full torque at low wheel speed", available_drive_torque(d, 10.0), 6000.0, 1e-9);
        check_near("power: P / omega at 100 rad/s", available_drive_torque(d, 100.0), 2000.0, 1e-9);
        check_near("power: continuous where the two limits meet", available_drive_torque(d, 200000.0 / 6000.0), 6000.0, 1e-9);
        check_near("power: rear drive uses the rear wheels' spin",
                   wheel_torques(d, pedals{1.0, 0.0, 0.0}, {0.0, 0.0, 100.0, 100.0})[RL].drive, 1000.0, 1e-9);
        check_near("power: half throttle, half of what is available",
                   wheel_torques(d, pedals{0.5, 0.0, 0.0}, {0.0, 0.0, 100.0, 100.0})[RL].drive, 500.0, 1e-9);
    }

    // the acceleration estimate behind the load transfer reaches each layout's traction limit
    {
        const Real W = 1200 * kGravity, L = 2.5, h = 0.5, lf = 1.3, lr = 1.2;
        auto limit = [&](Real split, Real throttle, Real brake)
        {
            const four_wheel_params p = flat_grip(split);
            return longitudinal_acceleration(p, wheel_torques(p.drive_brake, pedals{throttle, brake, 0}, still), 10.0) * 1200;
        };
        check_near("traction estimate: RWD, mu W lf / (L - mu h)", limit(0.0, 1.0, 0.0), W * lf / (L - h), 1e-3 * W);
        check_near("traction estimate: FWD, mu W lr / (L + mu h)", limit(1.0, 1.0, 0.0), W * lr / (L + h), 1e-3 * W);
        check_near("traction estimate: AWD, mu W", limit(0.5, 1.0, 0.0), W, 1e-3 * W);
        check_near("braking estimate: all four locked, -mu W", limit(0.0, 0.0, 1.0), -W, 1e-3 * W);
        check_near("small torques pass unchanged", limit(0.0, 0.05, 0.0), 0.05 * 6000 / 0.315, 1e-9);
    }

    // full throttle from 4 m/s: the car accelerates at its layout's traction limit. Undriven wheels still have
    // to be spun up by their tyres, so their inertia adds 2 I / R^2 to the mass; driven wheels are spinning.
    {
        const Real W = 1200 * kGravity, L = 2.5, h = 0.5, lf = 1.3, lr = 1.2, R = 0.315, I = 1.2;
        const Real free_pair = 2 * I / (R * R);
        struct layout
        {
            const char *name;
            Real split, expected;
        };
        // AWD with 30% of the torque on the front: about the front's share of the load while accelerating at 1 g,
        // so neither axle runs out of grip before the other and the car reaches mu g
        const layout layouts[] = {{"RWD", 0.0, W * lf / (L - h) / (1200 + free_pair)},
                                  {"FWD", 1.0, W * lr / (L + h) / (1200 + free_pair)},
                                  {"AWD 30/70", 0.3, kGravity}};
        for (const layout &c : layouts)
        {
            const Real a = full_throttle_acceleration(flat_grip(c.split));
            std::printf("full throttle %s: %.3f m/s^2 (traction limit %.3f)\n", c.name, a, c.expected);
            char what[96];
            std::snprintf(what, sizeof what, "acceleration: %s at its traction limit within 5%%", c.name);
            check_near(what, a, c.expected, 0.05 * c.expected);
        }
        check_true("acceleration: AWD > RWD > FWD", full_throttle_acceleration(flat_grip(0.3)) > full_throttle_acceleration(flat_grip(0.0)) &&
                                                        full_throttle_acceleration(flat_grip(0.0)) > full_throttle_acceleration(flat_grip(1.0)));

        // 50/50 puts too much torque on the front, which unloads when accelerating: the fronts spin, and with open
        // differentials the spinning wheels take the engine's speed up and its torque down. Still more than RWD,
        // but well short of mu g (limited-slip differentials, in phase 7, fix that).
        const Real even = full_throttle_acceleration(flat_grip(0.5));
        std::printf("full throttle AWD 50/50: %.3f m/s^2\n", even);
        check_true("acceleration: AWD 50/50 beats RWD but not the 30/70 split", even > full_throttle_acceleration(flat_grip(0.0)) &&
                                                                              even < 0.9 * kGravity);
    }

    // at speed the engine runs out of power: a = P / (m_eff v), where every wheel's spin inertia adds to the
    // mass (m_eff = m + 4 I / R^2). AWD at 25 m/s: the tyres could take more, the engine cannot give it.
    {
        const four_wheel_params p = flat_grip(0.5);
        four_wheel_state s = four_wheel_initial(p, 25.0);
        const four_wheel_input in{0.0, 1.0, 0.0, 0.0};
        for (int i = 0; i < 200; ++i)
            step(s, p, in, dt);
        const Real u0 = velocity_in_body(s.body).u;
        for (int i = 0; i < 100; ++i)
            step(s, p, in, dt);
        const Real u1 = velocity_in_body(s.body).u;
        const Real a = (u1 - u0) / 0.1;
        const Real expected = 200000.0 / ((1200 + 4 * 1.2 / (0.315 * 0.315)) * 0.5 * (u0 + u1));
        std::printf("full throttle AWD at %.1f m/s: %.3f m/s^2 (P / m_eff v = %.3f)\n", 0.5 * (u0 + u1), a, expected);
        // the driven wheels slip a little, so a little of the power goes into the sliding rubber, not into speed
        check_near("power: a = P / (m_eff v) at speed, within 3%", a, expected, 0.03 * expected);
    }

    // the launch: the wheels spin up, but only until the engine's power runs out. Without the power limit they
    // sped up for ever (a fixed torque at any wheel speed); now the spin peaks and then falls as the car gains speed.
    {
        const four_wheel_run r = four_wheel_launch();
        four_wheel_state s = r.initial;
        Real early = 0, late = 0; // largest wheel surface speed ahead of the car, before and after 2.5 s
        for (long i = 0; i < r.steps; ++i)
        {
            step(s, r.params, driver_input(r, s, i * r.dt), r.dt);
            for (const wheel_state &w : s.wheels)
            {
                const Real ahead = w.omega * r.params.wheel.radius - velocity_in_body(s.body).u;
                Real &worst = (i * r.dt < 2.5) ? early : late;
                worst = std::max(worst, ahead);
            }
        }
        std::printf("launch: wheel surface at most %.2f m/s ahead of the car until 2.5 s, %.2f m/s after\n", early, late);
        check_true("launch: the wheelspin settles instead of growing", late <= early);
        check_true("launch: and stays below 20 m/s", early < 20.0);
    }

    // straight braking from 20 m/s with the brake fully down: all four wheels lock, the car stops in
    // v^2 / (2 mu g), no wheel ever turns backwards, and the car does not swerve
    {
        const four_wheel_params p = flat_grip();
        four_wheel_state s = four_wheel_initial(p, 20.0);
        const four_wheel_input in{0.0, 0.0, 1.0, 0.0};
        bool never_backwards = true, all_locked = false;
        long steps = 0;
        while (velocity_in_body(s.body).u > 0 && steps < 100000)
        {
            step(s, p, in, dt);
            ++steps;
            bool locked = true;
            for (const wheel_state &w : s.wheels)
            {
                never_backwards = never_backwards && w.omega >= 0;
                locked = locked && w.omega == 0;
            }
            all_locked = all_locked || locked;
        }
        const Real expected = 20.0 * 20.0 / (2 * kGravity);
        std::printf("full brake from 20 m/s: %.3f m in %.3f s (v^2 / 2 mu g = %.3f m)\n", s.body.pos.x, steps * dt, expected);
        check_near("braking: distance = v^2 / (2 mu g) within 3%", s.body.pos.x, expected, 0.03 * expected);
        check_true("braking: all four wheels lock", all_locked);
        check_true("braking: no wheel ever turns backwards", never_backwards);
        check_near("braking: straight line", s.body.pos.y, 0.0, 1e-9);

        // weaker brakes: with 70% front bias the fronts lock first, the rears keep rolling
        four_wheel_params soft = p;
        soft.drive_brake.max_brake_torque = 4000; // measured: fronts locked within 1 s from about 3900 N m, rears still rolling at 4100
        four_wheel_state t = four_wheel_initial(soft, 20.0);
        for (int i = 0; i < 1000; ++i)
            step(t, soft, in, dt);
        check_true("bias: with weaker brakes the fronts lock and the rears still turn",
                   t.wheels[FL].omega == 0 && t.wheels[RL].omega > 0);
    }

    // handbrake turn: a full pull in a gentle left turn. The rear wheels lock, the rear axle loses its side grip
    // and the car rotates faster. Checked until just after the release (the scripted driver does not countersteer,
    // so the car goes on to spin; the handbrake scenario uses a gentler pull and recovers).
    {
        four_wheel_run with = four_wheel_handbrake();
        with.script.keys = {{0.0, four_wheel_input{}},
                            {1.0, four_wheel_input{}},
                            {1.3, four_wheel_input{0.05, 0.0, 0.0, 0.0}},
                            {2.5, four_wheel_input{0.05, 0.0, 0.0, 0.0}},
                            {2.55, four_wheel_input{0.05, 0.0, 0.0, 1.0}},
                            {3.1, four_wheel_input{0.05, 0.0, 0.0, 1.0}},
                            {3.15, four_wheel_input{0.05, 0.0, 0.0, 0.0}}};
        four_wheel_run without = with;
        for (auto &k : without.script.keys)
            k.value.handbrake = 0;

        four_wheel_state a = with.initial, b = without.initial;
        Real worst_beta = 0, max_r_with = 0, max_r_without = 0;
        bool rears_locked = false, fronts_turning = true;
        for (long i = 0; i < 3300; ++i) // until 3.3 s: just after the release
        {
            const Real t = i * dt;
            step(a, with.params, driver_input(with, a, t), dt);
            step(b, without.params, driver_input(without, b, t), dt);
            if (t > 2.6 && t < 3.1)
            {
                rears_locked = rears_locked || (a.wheels[RL].omega == 0 && a.wheels[RR].omega == 0);
                fronts_turning = fronts_turning && a.wheels[FL].omega > 0;
            }
            const body_velocity va = velocity_in_body(a.body);
            worst_beta = std::max(worst_beta, std::fabs(std::atan2(va.v, va.u)));
            max_r_with = std::max(max_r_with, a.body.omega.z);
            max_r_without = std::max(max_r_without, b.body.omega.z);
        }
        std::printf("handbrake: peak yaw rate %.3f rad/s (without %.3f), peak sideslip %.1f deg\n",
                    max_r_with, max_r_without, worst_beta * 180 / 3.14159265358979);
        check_true("handbrake: the rear wheels lock", rears_locked);
        check_true("handbrake: the front wheels keep turning", fronts_turning);
        check_true("handbrake: the car rotates at least 30% faster than without it", max_r_with > 1.3 * max_r_without);
        check_true("handbrake: the tail swings out (sideslip over 5 degrees)", worst_beta > 5 * 3.14159265358979 / 180);
    }

    // the handbrake scenario: the rear steps out a little and the car recovers, still going forward
    {
        const four_wheel_run r = four_wheel_handbrake();
        four_wheel_state s = r.initial;
        Real peak_beta = 0;
        for (long i = 0; i < r.steps; ++i)
        {
            step(s, r.params, driver_input(r, s, i * r.dt), r.dt);
            const body_velocity v = velocity_in_body(s.body);
            peak_beta = std::max(peak_beta, std::fabs(std::atan2(v.v, v.u)));
        }
        const body_velocity v = velocity_in_body(s.body);
        std::printf("handbrake flick: peak sideslip %.1f deg, at the end %.1f deg\n", peak_beta * 180 / 3.14159265358979,
                    std::atan2(v.v, v.u) * 180 / 3.14159265358979);
        check_true("handbrake flick: the rear steps out (over 4 degrees)", peak_beta > 4 * 3.14159265358979 / 180);
        check_true("handbrake flick: and the car recovers (under 2 degrees at the end, going forward)",
                   std::fabs(std::atan2(v.v, v.u)) < 2 * 3.14159265358979 / 180 && v.u > 10);
    }

    // the speed controller brakes when the car is too fast
    {
        four_wheel_run r;
        r.initial = four_wheel_initial(r.params, 20.0);
        r.hold_speed = 15.0;
        r.script.keys = {{0.0, four_wheel_input{}}};
        r.steps = 5000;
        check_true("speed hold: brakes at first", driver_input(r, r.initial, 0.0).brake > 0 && driver_input(r, r.initial, 0.0).throttle == 0);
        check_near("speed hold: settles at the target", velocity_in_body(run(r).body).u, 15.0, 0.05);
    }

    return finish("four_wheel_driving");
}
