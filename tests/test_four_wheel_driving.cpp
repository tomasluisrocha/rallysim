#include <algorithm>
#include <cmath>
#include <cstdio>
#include "scenarios/scenario_file.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// Phase 4, step 4: pedals, drive layouts, brakes and handbrake. The straight-line tests use load sensitivity off,
// so the closed forms (braking v^2 / 2 mu g, acceleration at the traction limit) are exact.
// Those forms are for a tyre that holds mu Fz while it slides, so flat_grip() is on the linear tyre; the phase 5
// tests at the end run the same experiments on the tyres that lose grip past their peak: peak_slide, and tmeasy, the
// car's own.

static four_wheel_params flat_grip(Real split_front = 0, tyre_curve curve = tyre_curve::linear)
{
    four_wheel_params p;
    p.front_tyre.load_sensitivity = 0;
    p.rear_tyre.load_sensitivity = 0;
    p.drive_brake.drive_split_front = split_front;
    p.set_tyre_curve(curve);
    return p;
}

// Straight braking from `speed` with the brake pedal held at `pedal`: the distance to a stop (m).
static Real stopping_distance(const four_wheel_params &p, Real speed, Real pedal)
{
    four_wheel_state s = four_wheel_initial(p, speed);
    const four_wheel_input in{0.0, 0.0, pedal, 0.0};
    for (long i = 0; velocity_in_body(s.body).u > 0 && i < 100000; ++i)
        step(s, p, in, 0.001);
    return s.body.pos.x;
}

// Braking from 20 m/s with the pedal squeezed in over 0.2 s to `pedal` and held: the average deceleration from
// 18 m/s down to 6 m/s (m/s^2), once the braking is established. Distance would hide the effect: the squeeze
// itself costs about 2 m, and stamped on from zero the front wheels lock before the load has moved forward.
static Real braking_deceleration(const four_wheel_params &p, Real pedal)
{
    four_wheel_state s = four_wheel_initial(p, 20.0);
    Real t18 = -1, t6 = -1;
    for (long i = 0; t6 < 0 && i < 100000; ++i)
    {
        const Real t = i * 0.001, u = velocity_in_body(s.body).u;
        if (t18 < 0 && u <= 18.0)
            t18 = t;
        if (u <= 6.0)
            t6 = t;
        step(s, p, four_wheel_input{0.0, 0.0, std::min(t / 0.2, 1.0) * pedal, 0.0}, 0.001);
    }
    return 12.0 / (t6 - t18);
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
        const four_wheel_run r = four_wheel_scenario("fw_launch");
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
        // On a tyre with a fall-off a spinning wheel's tyre pushes back with only its sliding share of the grip, so
        // the wheels spin up further than on the linear tyre before the power limit holds them: about 23 m/s ahead on
        // peak_slide, 27 on the car's tmeasy tyre (its slower fall leaves less push at this much slip).
        check_true("launch: the wheelspin settles instead of growing", late <= early);
        check_true("launch: and stays below 30 m/s", early < 30.0);
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
        four_wheel_run with = four_wheel_scenario("fw_handbrake");
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
                    max_r_with, max_r_without, worst_beta * 180 / kPi);
        check_true("handbrake: the rear wheels lock", rears_locked);
        check_true("handbrake: the front wheels keep turning", fronts_turning);
        check_true("handbrake: the car rotates at least 30% faster than without it", max_r_with > 1.3 * max_r_without);
        check_true("handbrake: the tail swings out (sideslip over 5 degrees)", worst_beta > 5 * kPi / 180);
    }

    // the handbrake scenario: the rear steps out a little and the car recovers, still going forward
    {
        const four_wheel_run r = four_wheel_scenario("fw_handbrake");
        four_wheel_state s = r.initial;
        Real peak_beta = 0;
        for (long i = 0; i < r.steps; ++i)
        {
            step(s, r.params, driver_input(r, s, i * r.dt), r.dt);
            const body_velocity v = velocity_in_body(s.body);
            peak_beta = std::max(peak_beta, std::fabs(std::atan2(v.v, v.u)));
        }
        const body_velocity v = velocity_in_body(s.body);
        std::printf("handbrake flick: peak sideslip %.1f deg, at the end %.1f deg\n", peak_beta * 180 / kPi,
                    std::atan2(v.v, v.u) * 180 / kPi);
        check_true("handbrake flick: the rear steps out (over 4 degrees)", peak_beta > 4 * kPi / 180);
        check_true("handbrake flick: and the car recovers (under 2 degrees at the end, going forward)",
                   std::fabs(std::atan2(v.v, v.u)) < 2 * kPi / 180 && v.u > 10);
    }

    // standstill (phase 4, step 5): the car stops and stays stopped, a parked car does not creep, and letting
    // go of the brake leaves it where it is
    {
        const four_wheel_params p; // load sensitivity on, as the car is normally run
        four_wheel_state s = four_wheel_initial(p, 20.0);
        const four_wheel_input brake{0.0, 0.0, 1.0, 0.0};
        long i = 0;
        while (velocity_in_body(s.body).u > 0 && i < 100000)
        {
            step(s, p, brake, dt);
            ++i;
        }
        const Real stopped_at = s.body.pos.x;
        const Real pitch_at_stop = pitch_angle(s.body.q); // nose down from the braking
        Real after = 0, back = 0, turned = 0;
        for (int k = 0; k < 3000; ++k)
        {
            step(s, p, brake, dt);
            if (k >= 500)
                after = std::max(after, std::fabs(velocity_in_body(s.body).u));
            back = std::max(back, stopped_at - s.body.pos.x);
            for (const wheel_state &w : s.wheels)
                turned += std::fabs(w.omega) * dt;
        }
        // two things move the CG back after the stop: the front tyre, still twisted by sigma_x times its capped slip
        // when it stops sliding (at rest the cap is the peak, the slip ratio where the force peaks, whatever the
        // curve), unwinds; and the body, pitched nose down by the braking, pitches back level, which moves the CG
        // back by about h * pitch
        const Real twist = p.front_tyre.relaxation_length_x * pure_slip_curve(p.front_tyre, 4000.0, slip_axis::x).peak_slip;
        const Real pitch_back = p.cg_height * pitch_at_stop;
        std::printf("standstill: after the stop |u| < %.6f m/s from 0.5 s on, rolled back %.4f m (twist %.4f + pitch %.4f), wheels turned %.6f rad\n",
                    after, back, twist, pitch_back, turned);
        check_true("standstill: the stopped car is below 1 cm/s after half a second", after < 0.01);
        check_true("standstill: it rolls back by the tyres' twist and the body pitching back, plus 30% overshoot at most",
                   back < 1.3 * (twist + pitch_back));
        // as the load moves back to the rear, the rear tyres briefly push harder than the rear brakes hold (30% bias)
        // and turn the wheels by a hair; then the brakes hold
        check_true("standstill: the braked wheels turn by less than 0.01 rad in all", turned < 0.01);

        // let go of the brake: the last few millimetres of twist unwind and the car stays put
        const Real released_at = s.body.pos.x;
        for (int k = 0; k < 3000; ++k)
            step(s, p, four_wheel_input{}, dt);
        std::printf("standstill: brake released, moved %.4f m, now at %.6f m/s\n", s.body.pos.x - released_at,
                    velocity_in_body(s.body).u);
        check_true("standstill: releasing the brake moves the car less than 1 cm", std::fabs(s.body.pos.x - released_at) < 0.01);
        check_true("standstill: and it ends at rest", std::fabs(velocity_in_body(s.body).u) < 1e-3);

        // parked with the brake on and a little throttle (less torque than the brake): no creep
        four_wheel_state parked = four_wheel_initial(p, 0.0);
        for (int k = 0; k < 5000; ++k)
            step(parked, p, four_wheel_input{0.0, 0.05, 0.3, 0.0}, dt);
        check_true("standstill: brake on, a little throttle, no creep", std::fabs(parked.body.pos.x) < 1e-3);
    }

    // the heading-hold driver (driver/heading_hold.hpp): its signs, then the hairpin it makes possible
    {
        const heading_hold_params c;
        // going straight and level, a target to the left: steer left; to the right: steer right
        check_true("heading hold: a target to the left steers left",
                   heading_hold_steer(body_velocity{10, 0, 0}, 0.0, 0.5, 1.3, 0.6, c) > 0);
        check_true("heading hold: a target to the right steers right",
                   heading_hold_steer(body_velocity{10, 0, 0}, 0.0, -0.5, 1.3, 0.6, c) < 0);
        // on target, but the car sliding with its velocity to the left of its heading: the wheels point along the
        // slide (left), which is countersteer when the tail has gone out to the right
        check_true("heading hold: on target and sliding, the wheels point along the slide (countersteer)",
                   heading_hold_steer(body_velocity{8, 3, 0}, 0.0, 0.0, 1.3, 0.6, c) > 0);
        // on target and rotating left: steer right to stop the rotation
        check_true("heading hold: rotating past the target, it steers against the rotation",
                   heading_hold_steer(body_velocity{8, 0, 1.0}, 0.0, 0.0, 1.3, 0.6, c) < 0);
        // the short way round: heading 170 degrees, target -170 is 20 degrees to the left, not 340 to the right
        check_near("heading hold: the error wraps the short way round", heading_error(-170 * kPi / 180, 170 * kPi / 180),
                   20 * kPi / 180, 1e-12);
        check_true("heading hold: never more than max_steer", std::fabs(heading_hold_steer(body_velocity{1, 5, 3}, 0.0, 3.0, 1.3, 0.6, c)) <= 0.6);
    }

    // the hairpin: a U-turn on the handbrake, the heading-hold driver catching the slide. It must turn the car round
    // (170 to 190 degrees), with a real slide (20 to 90 degrees of sideslip, not a spin), and end straight and at
    // speed. On both tyres with a fall-off: an open-loop script landed this only on a knife edge of the pull
    // (0.73 turned 83 degrees, 0.75 spun), so a driver that reacts is what the scenario is about.
    for (tyre_curve curve : {tyre_curve::tmeasy, tyre_curve::peak_slide})
    {
        four_wheel_run r = four_wheel_scenario("fw_hairpin");
        r.params.set_tyre_curve(curve);
        r.initial = four_wheel_initial(r.params, 8.0);
        four_wheel_state s = r.initial;
        Real turned = 0, previous = yaw_angle(s.body.q), peak_beta = 0;
        for (long i = 0; i < r.steps; ++i)
        {
            step(s, r.params, driver_input(r, s, i * r.dt), r.dt);
            const Real yaw = yaw_angle(s.body.q);
            turned += heading_error(yaw, previous); // the change since the last step, the short way
            previous = yaw;
            const body_velocity v = velocity_in_body(s.body);
            if (std::hypot(v.u, v.v) > 1)
                peak_beta = std::max(peak_beta, std::fabs(std::atan2(v.v, v.u)));
        }
        const body_velocity v = velocity_in_body(s.body);
        const char *name = curve == tyre_curve::tmeasy ? "tmeasy" : "peak_slide";
        std::printf("hairpin (%s): turned %.1f deg, peak sideslip %.1f deg, at the end sideslip %.2f deg at %.2f m/s\n", name,
                    turned * 180 / kPi, peak_beta * 180 / kPi, std::atan2(v.v, v.u) * 180 / kPi, v.u);
        char what[200];
        std::snprintf(what, sizeof what, "hairpin (%s): the car turns round, 170 to 190 degrees", name);
        check_true(what, std::fabs(turned) > 170 * kPi / 180 && std::fabs(turned) < 190 * kPi / 180);
        std::snprintf(what, sizeof what, "hairpin (%s): a real slide, 20 to 90 degrees of sideslip, not a spin", name);
        check_true(what, peak_beta > 20 * kPi / 180 && peak_beta < 90 * kPi / 180);
        std::snprintf(what, sizeof what, "hairpin (%s): it drives off straight (under 2 degrees) at the held speed", name);
        check_true(what, std::fabs(std::atan2(v.v, v.u)) < 2 * kPi / 180 && std::fabs(v.u - 8.0) < 0.1);
    }

    // the speed controller brakes when the car is too fast
    {
        four_wheel_run r;
        r.initial = four_wheel_initial(r.params, 20.0);
        r.hold_speed = constant_script<Real>(15.0);
        r.script.keys = {{0.0, four_wheel_input{}}};
        r.steps = 5000;
        check_true("speed hold: brakes at first", driver_input(r, r.initial, 0.0).brake > 0 && driver_input(r, r.initial, 0.0).throttle == 0);
        check_near("speed hold: settles at the target", velocity_in_body(run(r).body).u, 15.0, 0.05);
    }

    // Phase 5: the same straight-line experiments on the tyres with a peak, peak_slide and tmeasy (the car's own),
    // which give their most force at a small slip (the peak) and only their sliding share of it when the wheel is
    // locked or spinning freely (along the wheel: 0.8 for peak_slide, 0.75 for tmeasy).
    for (tyre_curve curve : {tyre_curve::peak_slide, tyre_curve::tmeasy})
    {
        const four_wheel_params p = flat_grip(0, curve);
        const char *name = curve == tyre_curve::peak_slide ? "peak_slide" : "tmeasy";
        const Real ratio = pure_slip_curve(p.front_tyre, p.front_tyre.nominal_load, slip_axis::x).sliding_ratio;
        char what[200];

        // locked braking: with the brake fully down every wheel locks, so the car decelerates at ratio mu g, not
        // mu g: the stop is 1 / ratio times longer than the linear tyre's v^2 / (2 mu g)
        const Real locked = stopping_distance(p, 20.0, 1.0);
        {
            const Real expected = 20.0 * 20.0 / (2 * ratio * kGravity);
            std::printf("%s, full brake from 20 m/s: %.3f m (v^2 / 2 sliding_ratio mu g = %.3f m; linear tyre %.3f m)\n",
                        name, locked, expected, 20.0 * 20.0 / (2 * kGravity));
            std::snprintf(what, sizeof what, "%s braking: locked, the distance is v^2 / (2 sliding_ratio mu g) within 3%%", name);
            check_near(what, locked, expected, 0.03 * expected);
        }

        // threshold braking: a pedal that keeps the tyres near their peak decelerates harder than locking them,
        // which gives only the sliding share of the grip. Once a wheel locks it stays locked (its tyre pushes back
        // with less than the brake holds), so the pedal is squeezed in, as a driver would, letting the load move
        // forward first. With the 70/30 bias the fronts lock first (about 3760 N each under braking holds 1185 N m
        // at the peak, against 2100 N m per unit of pedal), so the best pedal is short of that and the rears do less
        // than they could.
        {
            const Real locked_a = braking_deceleration(p, 1.0);
            Real best = locked_a, best_pedal = 1.0;
            for (int k = 40; k <= 100; k += 2)
            {
                const Real a = braking_deceleration(p, k / 100.0);
                if (a > best)
                {
                    best = a;
                    best_pedal = k / 100.0;
                }
            }
            std::printf("%s, braking from 18 to 6 m/s: locked %.3f m/s^2 (%.3f g), best pedal %.2f gives %.3f m/s^2 (%.3f g)\n",
                        name, locked_a, locked_a / kGravity, best_pedal, best, best / kGravity);
            std::snprintf(what, sizeof what, "%s braking: locked, the deceleration is sliding_ratio g within 3%%", name);
            check_near(what, locked_a, ratio * kGravity, 0.03 * ratio * kGravity);
            std::snprintf(what, sizeof what, "%s braking: the best pedal decelerates at over 0.9 g, harder than locking", name);
            check_true(what, best > 0.9 * kGravity);
            std::snprintf(what, sizeof what, "%s braking: but no harder than every tyre at its peak (g)", name);
            check_true(what, best < 1.01 * kGravity);
            std::snprintf(what, sizeof what, "%s braking: the best pedal is short of full", name);
            check_true(what, best_pedal < 1.0);
        }

        // traction, the mirror of braking: on the rear-drive car full throttle spins the rear wheels past their
        // peak, and once spinning they stay spinning; a throttle squeezed in to keep them near their peak
        // accelerates harder. Both are squeezed in over 0.3 s from 4 m/s; the acceleration is averaged from 0.5 to
        // 1.3 s.
        {
            auto acceleration = [](const four_wheel_params &car, Real throttle)
            {
                four_wheel_state s = four_wheel_initial(car, 4.0);
                Real u0 = 0;
                for (int i = 0; i < 1300; ++i)
                {
                    if (i == 500)
                        u0 = velocity_in_body(s.body).u;
                    step(s, car, four_wheel_input{0.0, std::min(i * 0.001 / 0.3, 1.0) * throttle, 0.0, 0.0}, 0.001);
                }
                return (velocity_in_body(s.body).u - u0) / 0.8;
            };
            const Real full = acceleration(p, 1.0);
            Real best = full, best_throttle = 1.0;
            for (int k = 20; k <= 100; k += 2)
            {
                const Real a = acceleration(p, k / 100.0);
                if (a > best)
                {
                    best = a;
                    best_throttle = k / 100.0;
                }
            }
            const Real linear_full = acceleration(flat_grip(0), 1.0);
            std::printf("%s, RWD from 4 m/s: full throttle %.3f m/s^2, best throttle %.2f gives %.3f m/s^2 (linear tyre, full throttle %.3f)\n",
                        name, full, best_throttle, best, linear_full);
            std::snprintf(what, sizeof what, "%s traction: full throttle spins the rears past the peak, below the linear tyre's", name);
            check_true(what, full < 0.9 * linear_full);
            std::snprintf(what, sizeof what, "%s traction: a lighter throttle accelerates more than 10%% harder than full throttle", name);
            check_true(what, best > 1.1 * full && best_throttle < 1.0);
            std::snprintf(what, sizeof what, "%s traction: but no harder than the linear tyre's traction limit", name);
            check_true(what, best < 1.01 * linear_full);
        }
    }

    return finish("four_wheel_driving");
}
