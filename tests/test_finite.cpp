#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include "core/finite.hpp"
#include "core/telemetry.hpp"
#include "driver/speed_hold.hpp"
#include "runner/scenario_file.hpp"
#include "runner/run_bicycle.hpp"
#include "runner/run_four_wheel.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// The non-finite guard (a NaN or infinity anywhere in the state fails the run): each state's
// checker names the first bad field, the runners stop at the step where a NaN appears, and the telemetry writer
// refuses a non-finite value instead of writing "-nan(ind)".

static const Real nan_value = std::numeric_limits<Real>::quiet_NaN();
static const Real inf_value = std::numeric_limits<Real>::infinity();

// Runs `f` and returns the message of the std::runtime_error it throws, or "" if it does not throw.
template <class F>
static std::string error_of(F f)
{
    try
    {
        f();
    }
    catch (const std::runtime_error &e)
    {
        return e.what();
    }
    return "";
}

static bool contains(const std::string &text, const std::string &part)
{
    return text.find(part) != std::string::npos;
}

int main()
{
    // the small checks: NaN and infinity both fail, ordinary numbers (zero, negative, huge) pass
    check_true("vec3: finite", is_finite(vec3{0, -3, 1e300}));
    check_true("vec3: NaN is not finite", !is_finite(vec3{0, nan_value, 0}));
    check_true("vec3: infinity is not finite", !is_finite(vec3{0, 0, -inf_value}));
    check_true("quat: finite", is_finite(quat{}));
    check_true("quat: NaN is not finite", !is_finite(quat{nan_value, 0, 0, 0}));

    // each state names its first bad field
    {
        rigid_body_state s;
        check_true("rigid body: a good state has no bad field", non_finite_field(s) == nullptr);
        s.vel.y = nan_value;
        s.omega.z = inf_value;
        const char *bad = non_finite_field(s);
        check_true("rigid body: the first bad field is named (vel before omega)", bad && std::string(bad) == "body.vel");
    }
    {
        four_wheel_state s = four_wheel_initial(four_wheel_params{}, 10.0);
        check_true("four-wheel: a good state has no bad field", non_finite_field(s) == nullptr);
        s.wheels[RL].omega = inf_value;
        const char *bad = non_finite_field(s);
        check_true("four-wheel: a bad wheel is named", bad && std::string(bad) == "wheel RL omega");
        s.body.q.x = nan_value;
        bad = non_finite_field(s);
        check_true("four-wheel: the body comes before the wheels", bad && std::string(bad) == "body.q");
    }
    {
        bicycle_state s;
        check_true("bicycle: a good state has no bad field", non_finite_field(s) == nullptr);
        s.v = nan_value;
        const char *bad = non_finite_field(s);
        check_true("bicycle: the bad field is named", bad && std::string(bad) == "v");
    }
    {
        const pedals p = speed_hold_pedals(1000, 10, 10, 1000, 0);
        check_true("speed hold: zero brake force at target leaves pedals off",
                   p.throttle == 0 && p.brake == 0 && std::isfinite(p.throttle) && std::isfinite(p.brake));
    }

    // require_finite: silent on a good state, and the message names the field, the step and the time
    {
        rigid_body_state s;
        check_true("require_finite: a good state passes", error_of([&] { require_finite(s, 12, 0.001); }).empty());
        s.pos.x = nan_value;
        const std::string e = error_of([&] { require_finite(s, 1500, 0.001); });
        std::printf("require_finite: \"%s\"\n", e.c_str());
        check_true("require_finite: names the field, the step and the time",
                   contains(e, "body.pos") && contains(e, "at step 1500") && contains(e, "t = 1.5"));
    }

    // a NaN injected into a run is caught at the step it first reaches the state: the steer turns NaN from the
    // first step after t = 0 (the script blends towards a NaN key), so the state is NaN after step 1 and the
    // runner stops there, not seconds later
    {
        four_wheel_run r = four_wheel_scenario("fw_circle");
        r.script.keys = {{0.0, four_wheel_input{}}, {0.5, four_wheel_input{nan_value, 0, 0, 0}}};
        const std::string e = error_of([&] { run(r); });
        std::printf("NaN steer: \"%s\"\n", e.c_str());
        check_true("run: a NaN steer stops the run with a non-finite state", contains(e, "non-finite state"));
        check_true("run: at the first step the NaN reaches the state (step 2)", contains(e, "at step 2 "));
    }
    {
        bicycle_run r;
        r.initial.v = 10;
        r.steps = 100;
        r.script.keys = {{0.0, bicycle_input{0.0, 0.0}}, {0.05, bicycle_input{0.0, inf_value}}};
        const std::string e = error_of([&] { run(r); });
        std::printf("infinite acceleration, bicycle: \"%s\"\n", e.c_str());
        check_true("run (bicycle): an infinite input stops the run", contains(e, "non-finite state"));
    }

    // outside_model: a springs car on its side, roof or below the ground stops the run; planar is never refused
    {
        const four_wheel_params p;
        four_wheel_run r;
        r.steps = 10;
        r.initial = four_wheel_initial(p, 0.0);
        check_true("outside_model: a level car is inside", outside_model(r.initial, r.params) == nullptr);

        four_wheel_pose pose;
        pose.position = {0, 0, 1};
        pose.roll = 3.14159265358979323846;
        r.initial = four_wheel_start(r.params, pose);
        const std::string roof = error_of([&] { run(r); });
        std::printf("roof: \"%s\"\n", roof.c_str());
        check_true("run: a car on its roof is refused", contains(roof, "outside the model") && contains(roof, "side or roof") &&
                                                            contains(roof, "at step 0 "));

        pose.roll = 0;
        pose.position = {0, 0, -0.1};
        r.initial = four_wheel_start(r.params, pose);
        const std::string low = error_of([&] { run(r); });
        std::printf("below ground: \"%s\"\n", low.c_str());
        check_true("run: a CG below the ground is refused", contains(low, "outside the model") && contains(low, "CG"));

        r.params.vertical = vertical_model::planar;
        check_true("outside_model: a planar car is never refused", outside_model(r.initial, r.params) == nullptr);
        pose.roll = 3.14159265358979323846;
        r.initial = four_wheel_start(r.params, pose);
        check_true("outside_model: a planar car upside down is not refused", outside_model(r.initial, r.params) == nullptr);
    }

    // every shipped scenario runs to its end inside the model
    for (const char *name : {"fw_showcase", "fw_brake", "fw_circle", "fw_lane", "fw_launch", "fw_hairpin", "fw_handbrake"})
    {
        const std::string e = error_of([&] { run(four_wheel_scenario(name)); });
        if (!e.empty())
            std::printf("%s: \"%s\"\n", name, e.c_str());
        const std::string what = std::string("scenario ") + name + " runs without throwing";
        check_true(what.c_str(), e.empty());
    }

    // the telemetry writer refuses a non-finite value and names the column and the row
    {
        const std::string path = "finite_test_output.csv";
        {
            telemetry_writer w(path, 0.001, {{"step", "count"}, {"speed", "m/s"}});
            check_true("telemetry: finite rows are written", error_of([&] { w.row({0, 1.0}); w.row({1, 2.0}); }).empty());
            const std::string e = error_of([&] { w.row({2, nan_value}); });
            std::printf("telemetry: \"%s\"\n", e.c_str());
            check_true("telemetry: a NaN is refused, naming the column and the row", contains(e, "speed") && contains(e, "row 2"));
        }
        std::remove(path.c_str());
    }

    return finish("finite");
}
