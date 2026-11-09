#include <algorithm>
#include <cmath>
#include <cstdio>
#include "scenarios/scenario_file.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// Phase 4, step 6: the four-wheel car on springs. Gravity acts, each corner sits on a spring and damper, the tyre
// forces act on the ground, so the body heaves, rolls and pitches and the load transfer comes out of the springs.
// Each check is against a closed form for this car: 1200 kg, h 0.5 m, lf 1.3, lr 1.2, tracks 1.5, 50 kN/m and
// 3800 N s/m at every corner.

static four_wheel_run steady_turn(const four_wheel_params &p, Real speed, Real steer, long steps)
{
    four_wheel_run r;
    r.params = p;
    r.initial = four_wheel_initial(p, speed);
    r.hold_speed = constant_script<Real>(speed);
    r.script.keys = {{0.0, four_wheel_input{steer, 0.0, 0.0, 0.0}}};
    r.steps = steps;
    return r;
}

static bool identical(const four_wheel_state &a, const four_wheel_state &b)
{
    if (!identical(a.body, b.body))
        return false;
    for (std::size_t i = 0; i < kWheels; ++i)
        if (a.wheels[i].omega != b.wheels[i].omega || a.wheels[i].slip_ratio != b.wheels[i].slip_ratio ||
            a.wheels[i].lateral_slip != b.wheels[i].lateral_slip)
            return false;
    return true;
}

int main()
{
    const four_wheel_params p; // the springs model is the default
    const Real dt = 0.001, m = 1200, h = 0.5, L = 2.5, t = 1.5, g = kGravity, k = 50000, c = 3800;

    // one corner: pushes, never pulls
    {
        const corner_params cp;
        const corner_load l = corner_force(cp, 0.05, 0.1);
        check_near("corner: spring = k * deflection", l.spring, 50000 * 0.05, 1e-9);
        check_near("corner: damper = c * deflection rate", l.damper, 3800 * 0.1, 1e-9);
        check_near("corner: load = spring + damper", l.fz, 2500 + 380, 1e-9);
        const corner_load off = corner_force(cp, -0.01, -0.5);
        check_true("corner: not compressed (wheel in the air), no load at all", off.fz == 0 && off.spring == 0 && off.damper == 0);
        const corner_load pulling = corner_force(cp, 0.001, -1.0);
        check_true("corner: a damper pulling harder than the spring pushes leaves the tyre unloaded, never pulled",
                   pulling.fz == 0 && pulling.spring + pulling.damper < 0);
    }

    // standing still: each spring compressed by its share of the weight, the car level at h, and it stays there
    {
        four_wheel_state s = four_wheel_initial(p, 0.0);
        const four_wheel_forces f = evaluate(s, p, four_wheel_input{});
        Real total = 0;
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            check_near("static: deflection = share of the weight / k", f.corner[i].deflection, p.static_load(i) / k, 1e-12);
            check_near("static: load = share of the weight", f.fz[i], p.static_load(i), 1e-6);
            total += f.fz[i];
        }
        check_near("static: the four loads carry the weight", total, m * g, 1e-6);
        check_near("static: front corner compressed by W lr / (2 L k)", f.corner[FL].deflection, m * g * 1.2 / 2.5 / 2 / k, 1e-12);

        for (int i = 0; i < 10000; ++i)
            step(s, p, four_wheel_input{}, dt);
        check_near("static: still at h after 10 s", s.body.pos.z, h, 1e-9);
        check_near("static: no roll", roll_angle(s.body.q), 0.0, 1e-9);
        check_near("static: no pitch", pitch_angle(s.body.q), 0.0, 1e-9);
        check_true("static: not moved along the ground", std::fabs(s.body.pos.x) < 1e-9 && std::fabs(s.body.pos.y) < 1e-9);
    }

    // heave: lifted by 1 cm and let go. Undamped it rings at sqrt(sum k / m) / 2 pi; damped (zeta = sum c / (2
    // sqrt(sum k m)), about 0.49) it undershoots by exp(-pi zeta / sqrt(1 - zeta^2)) of the lift. The front and
    // rear springs sit at different distances from the CG, so heave and pitch couple a little.
    {
        four_wheel_params undamped = p;
        undamped.front_corner.damper_rate = undamped.rear_corner.damper_rate = 0;
        four_wheel_state s = four_wheel_initial(undamped, 0.0);
        s.body.pos.z += 0.01;
        Real crossings[3] = {0, 0, 0};
        int found = 0;
        Real before = s.body.pos.z - h;
        for (int i = 1; i <= 3000 && found < 3; ++i)
        {
            step(s, undamped, four_wheel_input{}, dt);
            const Real now = s.body.pos.z - h;
            if ((before > 0) != (now > 0))
                crossings[found++] = i * dt - dt * now / (now - before); // linear interpolation between the steps
            before = now;
        }
        const Real f_heave = 1 / (crossings[2] - crossings[0]);
        const Real expected = std::sqrt(4 * k / m) / (2 * kPi);
        std::printf("heave, undamped: %.4f Hz (sqrt(sum k / m) / 2 pi = %.4f Hz)\n", f_heave, expected);
        check_true("heave: three zero crossings found", found == 3);
        check_near("heave: undamped frequency within 2%", f_heave, expected, 0.02 * expected);

        four_wheel_state d = four_wheel_initial(p, 0.0);
        d.body.pos.z += 0.01;
        Real lowest = 0;
        for (int i = 0; i < 1500; ++i)
        {
            step(d, p, four_wheel_input{}, dt);
            lowest = std::min(lowest, d.body.pos.z - h);
        }
        const Real zeta = 4 * c / (2 * std::sqrt(4 * k * m));
        const Real undershoot = std::exp(-kPi * zeta / std::sqrt(1 - zeta * zeta));
        std::printf("heave, damped: undershoot %.3f of the lift (zeta %.3f, exp(-pi zeta / sqrt(1 - zeta^2)) = %.3f)\n",
                    -lowest / 0.01, zeta, undershoot);
        check_near("heave: damped undershoot within 15%", -lowest / 0.01, undershoot, 0.15 * undershoot);
        check_near("heave: settles back at h", d.body.pos.z, h, 1e-5);
    }

    // steady left turn at 20 m/s and about 4 m/s^2: the body rolls out of the turn (positive roll, right side down)
    // by m a_y h / (K_roll - m g h), K_roll = (k t_f^2 + k t_r^2) / 2: the springs resist, the CG leaning out helps.
    // The loads balance the roll moment: (right - left) * t / 2 = m h (a_y + g roll), split between the axles by
    // their roll stiffness (equal here, so 50/50).
    {
        const four_wheel_run r = steady_turn(p, 20.0, 0.0277, 8000);
        const four_wheel_state s = run(r);
        const body_velocity v = velocity_in_body(s.body);
        const Real a_y = std::hypot(v.u, v.v) * v.r;
        const Real roll = roll_angle(s.body.q);
        const Real k_roll = (k * t * t + k * t * t) / 2;
        const Real expected_roll = m * a_y * h / (k_roll - m * g * h);
        std::printf("steady turn: a_y %.3f m/s^2, roll %.3f deg (m a_y h / (K - m g h) = %.3f deg)\n", a_y,
                    roll * 180 / kPi, expected_roll * 180 / kPi);
        check_true("roll: positive (right side down) in a left turn", roll > 0);
        check_near("roll: angle within 3% of m a_y h / (K_roll - m g h)", roll, expected_roll, 0.03 * expected_roll);

        const four_wheel_forces f = evaluate(s, p, driver_input(r, s, r.steps * r.dt));
        const Real right = f.fz[FR] + f.fz[RR], left = f.fz[FL] + f.fz[RL];
        const Real moment = (right - left) * t / 2;
        const Real expected_moment = m * h * (a_y + g * roll);
        std::printf("steady turn: load moment %.0f N m (m h (a_y + g roll) = %.0f N m; without the lean, m a_y h = %.0f)\n",
                    moment, expected_moment, m * a_y * h);
        check_near("load transfer: (right - left) t / 2 = m h (a_y + g roll), within 2%", moment, expected_moment, 0.02 * expected_moment);
        check_near("load transfer: equal springs share it 50/50 between the axles",
                   (f.fz[FR] - f.fz[FL]) / (right - left), 0.5, 0.02);
        check_near("load transfer: the four loads still carry the weight", right + left, m * g, 0.002 * m * g);
    }

    // steady braking at about 0.45 g (no wheel locks): the body pitches nose down (positive) by
    // m a_x h / (K_pitch - m g h), K_pitch = 2 (k lf^2 + k lr^2), and the front gains m h (a_x + g pitch) / L
    {
        four_wheel_state s = four_wheel_initial(p, 25.0);
        const four_wheel_input brake{0.0, 0.0, 0.3, 0.0};
        for (int i = 0; i < 800; ++i)
            step(s, p, brake, dt);
        const Real u0 = velocity_in_body(s.body).u;
        for (int i = 0; i < 400; ++i)
            step(s, p, brake, dt);
        const Real decel = (u0 - velocity_in_body(s.body).u) / 0.4;
        const Real pitch = pitch_angle(s.body.q);
        const Real k_pitch = 2 * (k * 1.3 * 1.3 + k * 1.2 * 1.2);
        const Real expected_pitch = m * decel * h / (k_pitch - m * g * h);
        std::printf("steady braking: %.3f m/s^2, pitch %.3f deg (m a h / (K - m g h) = %.3f deg)\n", decel,
                    pitch * 180 / kPi, expected_pitch * 180 / kPi);
        check_true("pitch: positive (nose down) under braking", pitch > 0);
        check_near("pitch: angle within 5% of m a_x h / (K_pitch - m g h)", pitch, expected_pitch, 0.05 * expected_pitch);

        const four_wheel_forces f = evaluate(s, p, brake);
        const Real front_gain = f.fz[FL] + f.fz[FR] - m * g * 1.2 / L;
        const Real expected_gain = m * h * (decel + g * pitch) / L;
        check_near("pitch: the front gains m h (a_x + g pitch) / L, within 3%", front_gain, expected_gain, 0.03 * expected_gain);
        for (const wheel_state &w : s.wheels)
            check_true("pitch: no wheel locked at this brake pressure", w.omega > 0);
    }

    // wheel lift: a high CG (0.85 m) and very stiff front springs (300 kN/m against 50 kN/m at the rear, so the
    // front carries 86% of the roll stiffness). In a left turn the front's share of the roll moment,
    // 0.86 m h (a_y + g roll) / t, outgrows the inner front's static load at about a_y = 5 m/s^2: that wheel lifts.
    // Its load reaches zero and never goes below, the other three stay down, and the car stays upright because the
    // front tyres run out of grip first (the car understeers) before the turn gets any harder. The steer is ramped
    // in; a sudden step of steer on this car overshoots, lifts the inner rear too and rolls it over.
    // On the linear tyre: this tests the springs, and the turn has to reach about 5 m/s^2, so it should not hang on
    // which tyre with a fall-off the car has. On peak_slide (phase 5, measured) the understeering front slides down
    // to its sliding share of the grip, and the turn tops out just short of lifting the wheel (lowest load about
    // 100 N).
    {
        four_wheel_params q = p;
        q.set_tyre_curve(tyre_curve::linear);
        q.front_corner.spring_rate = 300000;
        q.rear_corner.spring_rate = 50000;
        q.cg_height = 0.85;
        four_wheel_run r;
        r.params = q;
        r.initial = four_wheel_initial(q, 18.0);
        r.hold_speed = constant_script<Real>(18.0);
        r.steps = 6000;
        r.script.keys = {{0.0, four_wheel_input{}}, {0.5, four_wheel_input{}},
                         {1.5, four_wheel_input{0.09, 0.0, 0.0, 0.0}}, {6.0, four_wheel_input{0.09, 0.0, 0.0, 0.0}}};
        four_wheel_state s = r.initial;
        Real lowest = 1e9, worst_roll = 0, lifted_for = 0;
        bool others_down = true;
        for (long i = 0; i < r.steps; ++i)
        {
            const four_wheel_input in = driver_input(r, s, i * r.dt);
            const four_wheel_forces f = evaluate(s, q, in);
            for (std::size_t w = 0; w < kWheels; ++w)
                lowest = std::min(lowest, f.fz[w]);
            if (f.fz[FL] == 0 && f.corner[FL].deflection <= 0)
                lifted_for += r.dt;
            others_down = others_down && f.fz[FR] > 0 && f.fz[RL] > 0 && f.fz[RR] > 0;
            worst_roll = std::max(worst_roll, std::fabs(roll_angle(s.body.q)));
            step(s, q, in, r.dt);
        }
        std::printf("lift: inner front off the ground for %.2f s, lowest load %.1f N, largest roll %.2f deg\n", lifted_for,
                    lowest, worst_roll * 180 / kPi);
        check_true("lift: the inner front wheel leaves the ground (load zero, spring extended) and stays up", lifted_for > 2.0);
        check_true("lift: no load is ever negative", lowest >= 0);
        check_true("lift: the other three wheels stay on the ground", others_down);
        check_true("lift: the car stays upright", worst_roll < 5 * kPi / 180);
    }

    // a right run mirrors a left run, roll included; the same run gives the same bits
    {
        four_wheel_run left = four_wheel_scenario("fw_lane");
        four_wheel_run right = left;
        for (auto &key : right.script.keys)
            key.value.steer = -key.value.steer;
        four_wheel_state a = left.initial, b = right.initial;
        Real worst_roll = 0;
        for (long i = 0; i < left.steps; ++i)
        {
            step(a, left.params, driver_input(left, a, i * left.dt), left.dt);
            step(b, right.params, driver_input(right, b, i * right.dt), right.dt);
            worst_roll = std::max(worst_roll, std::fabs(roll_angle(a.body.q)));
        }
        check_near("mirror: y", a.body.pos.y, -b.body.pos.y, 1e-9);
        check_near("mirror: roll", roll_angle(a.body.q), -roll_angle(b.body.q), 1e-9);
        check_near("mirror: pitch", pitch_angle(a.body.q), pitch_angle(b.body.q), 1e-9);
        check_near("mirror: height", a.body.pos.z, b.body.pos.z, 1e-9);
        check_true("mirror: the body did roll in the lane change", worst_roll > 0.002);
        check_true("determinism: two runs give identical bits", identical(a, run(left)));
    }

    return finish("four_wheel_springs");
}
