#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "vehicle/quarter_car.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// One corner with the default parameters: rest, undamped modes, a road step against RK4, energy, wheel lift.
namespace
{
    const quarter_car_params P;
    const Real g = kGravity;
    const Real dt = 1e-3;

    // Frequency (Hz) from the first to the last zero crossing of zs about rest_zs.
    Real crossing_frequency(quarter_car_state s, const quarter_car_params &p, Real rest_zs, Real seconds, int &n)
    {
        Real first = 0, last = 0;
        n = 0;
        Real before = s.zs - rest_zs;
        const int steps = static_cast<int>(std::lround(seconds / dt));
        for (int i = 1; i <= steps; ++i)
        {
            step(s, p, 0, 0, dt);
            const Real now = s.zs - rest_zs;
            if ((before > 0) != (now > 0))
            {
                last = i * dt - dt * now / (now - before);
                if (n++ == 0)
                    first = last;
            }
            before = now;
        }
        return n > 1 ? (n - 1) / (2 * (last - first)) : 0;
    }

    // Classical RK4 on the four linear equations; does not use evaluate or step.
    struct y4
    {
        Real a[4];
    };
    y4 deriv(const y4 &y, Real road)
    {
        const Real strut = P.spring_rate * (y.a[2] - y.a[0]) + P.damper_rate * (y.a[3] - y.a[1]);
        const Real tyre = P.tyre_rate * (road - y.a[2]);
        return {{y.a[1], strut / P.sprung_mass - g, y.a[3], (tyre - strut) / P.unsprung_mass - g}};
    }
    y4 axpy(const y4 &y, const y4 &d, Real h)
    {
        y4 r;
        for (int i = 0; i < 4; ++i)
            r.a[i] = y.a[i] + h * d.a[i];
        return r;
    }
    void rk4(y4 &y, Real road, Real h)
    {
        const y4 k1 = deriv(y, road);
        const y4 k2 = deriv(axpy(y, k1, h / 2), road);
        const y4 k3 = deriv(axpy(y, k2, h / 2), road);
        const y4 k4 = deriv(axpy(y, k3, h), road);
        for (int i = 0; i < 4; ++i)
            y.a[i] += h / 6 * (k1.a[i] + 2 * k2.a[i] + 2 * k3.a[i] + k4.a[i]);
    }
}

int main()
{
    const Real ms = P.sprung_mass, mu = P.unsprung_mass, ks = P.spring_rate, kt = P.tyre_rate;
    const quarter_car_state rest0 = quarter_car_rest(P, 0);

    // rest
    {
        quarter_car_state s = rest0;
        const quarter_car_forces f = evaluate(s, P, 0, 0);
        check_near("rest: spring compression = ms g / ks", s.zu - s.zs, ms * g / ks, 1e-12);
        check_near("rest: spring compression value", s.zu - s.zs, 0.051012, 1e-12);
        check_near("rest: tyre compression = (ms + mu) g / kt", 0 - s.zu, (ms + mu) * g / kt, 1e-12);
        check_near("rest: tyre compression value", 0 - s.zu, 0.011772, 1e-12);
        check_near("rest: tyre force = (ms + mu) g", f.tyre, (ms + mu) * g, 1e-6);
        for (int i = 0; i < 10000; ++i)
            step(s, P, 0, 0, dt);
        const Real moved = std::max({std::fabs(s.zs - rest0.zs), std::fabs(s.zu - rest0.zu), std::fabs(s.vs), std::fabs(s.vu)});
        std::printf("rest: largest drift after 10 s %.3g\n", moved);
        check_true("rest: nothing moved by more than 1e-9 in 10 s", moved <= 1e-9);
    }

    // undamped modes
    {
        quarter_car_params p = P;
        p.damper_rate = 0;
        const Real a = ks / ms, b = (ks + kt) / mu;
        const Real root = std::sqrt((a + b) * (a + b) - 4 * ks * kt / (ms * mu));
        const Real w2[2] = {((a + b) - root) / 2, ((a + b) + root) / 2};
        const char *names[2] = {"body mode", "wheel mode"};
        for (int m = 0; m < 2; ++m)
        {
            const Real expected = std::sqrt(w2[m]) / (2 * kPi);
            quarter_car_state s = rest0;
            // mode shape zu = zs (1 - ms w^2 / ks); the wheel mode has 5 mm at the wheel (5 mm at the body would
            // swing the wheel 0.19 m, off the road)
            const Real ratio = 1 - ms * w2[m] / ks;
            s.zs += m == 0 ? 0.005 : 0.005 / ratio;
            s.zu += m == 0 ? 0.005 * ratio : 0.005;
            int n = 0;
            const Real f = crossing_frequency(s, p, rest0.zs, 3.0, n);
            std::printf("%s: %.4f Hz (closed form %.4f Hz), off by %.3g relative, %d crossings\n", names[m], f, expected,
                        std::fabs(f - expected) / expected, n);
            check_true("modes: enough zero crossings", n > 2);
            check_near(names[m], f, expected, 0.01 * expected);
        }
    }

    // road step against RK4 at 1e-5
    {
        const Real road = 0.02, h = 1e-5;
        const int total = 200000; // 2 s
        std::vector<Real> ref(total + 1);
        y4 y = {{rest0.zs, 0, rest0.zu, 0}};
        ref[0] = y.a[0];
        for (int i = 1; i <= total; ++i)
        {
            rk4(y, road, h);
            ref[i] = y.a[0];
        }
        Real peak = 0, peak_t = 0;
        for (int i = 0; i <= total; ++i)
            if (ref[i] - rest0.zs > peak)
            {
                peak = ref[i] - rest0.zs;
                peak_t = i * h;
            }
        std::printf("road step: reference peak rise %.4f mm at %.4f s\n", peak * 1e3, peak_t);
        check_near("road step: reference peak rise 28.50 mm", peak, 0.02850, 0.05e-3);
        check_near("road step: reference peak time 0.184 s", peak_t, 0.184, 0.002);

        Real err[2] = {0, 0};
        const Real steps_dt[2] = {1e-3, 5e-4};
        for (int k = 0; k < 2; ++k)
        {
            quarter_car_state s = rest0;
            const int stride = static_cast<int>(std::lround(steps_dt[k] / h));
            const int n = static_cast<int>(std::lround(2.0 / steps_dt[k]));
            for (int i = 1; i <= n; ++i)
            {
                step(s, P, road, 0, steps_dt[k]);
                err[k] = std::max(err[k], std::fabs(s.zs - ref[i * stride]));
            }
        }
        std::printf("road step: largest zs error %.4f mm at 1 ms, %.4f mm at 0.5 ms (ratio %.3f)\n", err[0] * 1e3,
                    err[1] * 1e3, err[1] / err[0]);
        check_true("road step: error under 0.2 mm at 1 ms", err[0] < 0.2e-3);
        check_true("road step: halving dt cuts the error to 0.4-0.6", err[1] > 0.4 * err[0] && err[1] < 0.6 * err[0]);
    }

    // energy, undamped and damped
    {
        const Real e_rest = quarter_car_energy(rest0, P, 0);
        quarter_car_state start = rest0;
        start.zs += 0.01;
        const Real e0 = quarter_car_energy(start, P, 0);
        const Real excess = e0 - e_rest;

        quarter_car_params und = P;
        und.damper_rate = 0;
        quarter_car_state s = start;
        Real lo = 0, hi = 0;
        for (int i = 0; i < 10000; ++i)
        {
            step(s, und, 0, 0, dt);
            const Real d = quarter_car_energy(s, und, 0) - e0;
            lo = std::min(lo, d);
            hi = std::max(hi, d);
        }
        std::printf("undamped energy: +%.2f%% / %.2f%% of the %.4f J above rest\n", hi / excess * 100, lo / excess * 100, excess);
        check_true("undamped energy within 2% of the excess", hi <= 0.02 * excess && -lo <= 0.02 * excess);

        s = start;
        Real prev = e0, worst_rise = -1e300;
        for (int i = 0; i < 4000; ++i)
        {
            step(s, P, 0, 0, dt);
            const Real e = quarter_car_energy(s, P, 0);
            worst_rise = std::max(worst_rise, e - prev);
            prev = e;
        }
        std::printf("damped energy: largest rise per step %.3g J, ends %.3g J from rest energy\n", worst_rise, prev - e_rest);
        check_true("damped energy: no step raises it by more than 1e-9 J", worst_rise <= 1e-9);
        check_near("damped energy ends at the rest energy", prev, e_rest, 1e-6);
    }

    // the wheel leaves the road
    {
        const Real road = -0.15;
        quarter_car_state s = rest0;
        const Real e0 = quarter_car_energy(s, P, road);
        int zero_steps = 0;
        bool negative = false;
        Real peak = 0, above = -1e300;
        for (int i = 0; i < 4000; ++i)
        {
            const Real tyre = evaluate(s, P, road, 0).tyre;
            if (tyre == 0)
                ++zero_steps;
            negative = negative || tyre < 0;
            peak = std::max(peak, tyre);
            step(s, P, road, 0, dt);
            above = std::max(above, quarter_car_energy(s, P, road) - e0);
        }
        const quarter_car_state end = quarter_car_rest(P, road);
        const Real tyre_end = evaluate(s, P, road, 0).tyre;
        std::printf("wheel leaves: %d zero-force steps, peak tyre force %.0f N, energy max %.3g J over start, "
                    "end errors %.3g / %.3g m, tyre %.4f N off\n",
                    zero_steps, peak, above, s.zs - end.zs, s.zu - end.zu, tyre_end - (ms + mu) * g);
        check_true("wheel leaves: tyre force is 0 for more than 100 steps", zero_steps > 100);
        check_true("wheel leaves: tyre force never negative", !negative);
        check_true("wheel leaves: energy never above its start", above <= 0);
        check_near("wheel leaves: zs ends at rest", s.zs, end.zs, 1e-6);
        check_near("wheel leaves: zu ends at rest", s.zu, end.zu, 1e-6);
        check_near("wheel leaves: tyre force ends at the weight", tyre_end, (ms + mu) * g, 0.01);
    }

    // tyre damping and a moving road (the default tyre has no damping, and every road above stands still)
    {
        quarter_car_params p = P;
        p.tyre_damping = 1000;

        // the force: spring plus damping against the road's own speed; never a pull
        quarter_car_state s = rest0;
        s.vu = 0.2;
        const Real spring = kt * (0 - s.zu);
        check_near("tyre damping: force = kt (road - zu) + ct (road_rate - vu)", evaluate(s, p, 0, 0.5).tyre,
                   spring + 1000 * (0.5 - 0.2), 1e-9);
        s.vu = 5; // the wheel leaving fast: the damping alone would pull by 5000 N, more than the spring's 2943 N
        check_true("tyre damping: a wheel leaving fast is not pulled back", evaluate(s, p, 0, 0).tyre == 0);

        // a road rising at a steady 0.5 m/s carries the corner up with it, compressed exactly as at rest
        // (the same motion seen from a moving frame); a road_rate left out of the force would compress it more
        const Real rate = 0.5;
        s = rest0;
        s.vs = s.vu = rate;
        Real worst = 0;
        for (int i = 0; i < 2000; ++i)
        {
            step(s, p, rate * i * dt, rate, dt);
            const Real road = rate * (i + 1) * dt;
            worst = std::max({worst, std::fabs((s.zu - s.zs) - ms * g / ks), std::fabs((road - s.zu) - (ms + mu) * g / kt)});
        }
        std::printf("rising road: compressions within %.3g m of rest over 2 s, body risen %.6f m\n", worst, s.zs - rest0.zs);
        check_true("rising road: spring and tyre stay compressed as at rest (1e-9 m)", worst <= 1e-9);
        check_near("rising road: the body rises at the road's speed", s.zs - rest0.zs, rate * 2.0, 1e-9);

        // damping in the tyre only takes energy out: lifted 10 mm and let go, as the damped check above
        s = rest0;
        s.zs += 0.01;
        Real prev = quarter_car_energy(s, p, 0), worst_rise = -1e300;
        for (int i = 0; i < 4000; ++i)
        {
            step(s, p, 0, 0, dt);
            const Real e = quarter_car_energy(s, p, 0);
            worst_rise = std::max(worst_rise, e - prev);
            prev = e;
        }
        std::printf("tyre damping, energy: largest rise per step %.3g J, ends %.3g J from rest energy\n", worst_rise,
                    prev - quarter_car_energy(rest0, p, 0));
        check_true("tyre damping: no step raises the energy by more than 1e-9 J", worst_rise <= 1e-9);
        check_near("tyre damping: ends at the rest energy", prev, quarter_car_energy(rest0, p, 0), 1e-6);
    }

    // non_finite_field
    {
        const Real nan = std::nan("");
        const char *names[4] = {"zs", "vs", "zu", "vu"};
        check_true("non_finite_field: finite state gives nullptr", non_finite_field(quarter_car_state{}) == nullptr);
        for (int i = 0; i < 4; ++i)
        {
            quarter_car_state s;
            (i == 0 ? s.zs : i == 1 ? s.vs : i == 2 ? s.zu : s.vu) = nan;
            const char *got = non_finite_field(s);
            check_true((std::string("non_finite_field: names a NaN in ") + names[i]).c_str(),
                       got != nullptr && std::string(got) == names[i]);
        }
    }

    return finish("quarter_car");
}
