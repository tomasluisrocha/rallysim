#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>
#include <vector>
#include "check.hpp"
#include "scenarios/four_wheel_driving.hpp"
#include "scenarios/scenario_file.hpp"

using namespace rallysim;

namespace
{
    four_wheel_run load(const std::string &text, const settings &overrides = {})
    {
        std::istringstream in(text);
        return load_scenario(in, "t.ini", overrides);
    }

    // The messages loading `text` throws, or none if it loads.
    std::vector<std::string> errors_of(const std::string &text, const settings &overrides = {})
    {
        try
        {
            load(text, overrides);
        }
        catch (const ini_error &e)
        {
            return e.messages;
        }
        return {};
    }

    // Exactly one error, starting with `at` ("t.ini:3:", "--set:", "t.ini:"), holding `words`.
    void check_one_error(const char *what, const std::string &text, const std::string &at, const std::string &words,
                         const settings &overrides = {})
    {
        const std::vector<std::string> e = errors_of(text, overrides);
        const bool ok = e.size() == 1 && e[0].rfind(at, 0) == 0 && e[0].find(words) != std::string::npos;
        if (!ok)
            for (const std::string &m : e)
                std::printf("  got: %s\n", m.c_str());
        check_true(what, ok);
    }

    bool same_vec(const vec3 &a, const vec3 &b) { return a.x == b.x && a.y == b.y && a.z == b.z; }

    bool same_tables(const tyre_params &a, const tyre_params &b)
    {
        return a.curve == b.curve && a.tmeasy_x.nominal.stiffness == b.tmeasy_x.nominal.stiffness &&
               a.tmeasy_x.double_load.stiffness == b.tmeasy_x.double_load.stiffness &&
               a.tmeasy_y.nominal.stiffness == b.tmeasy_y.nominal.stiffness &&
               a.tmeasy_y.double_load.stiffness == b.tmeasy_y.double_load.stiffness;
    }

    const std::string kMinimal = "[run]\nduration = 1 s\n";
}

int main()
{
    // ---- a file gives the same run as the built-in scenario -------------------------------------------------
    {
        four_wheel_run file = load("[scenario]\n"
                                         "name = fw_brake\n"
                                         "description = straight-line braking from 25 m/s\n"
                                         "[run]\n"
                                         "model = four_wheel\n"
                                         "duration = 5 s\n"
                                         "[start]\n"
                                         "speed = 90 km/h\n");
        four_wheel_run built = four_wheel_brake();
        check_true("same: dt and steps", file.dt == built.dt && file.steps == built.steps);
        bool params_same = true;
        for (const param_entry &e : four_wheel_registry())
            params_same = params_same && e.field(file.params) == e.field(built.params);
        check_true("same: every registry parameter, exactly", params_same);
        check_true("same: tyre tables, bit for bit (no refit of an untouched car)",
                   same_tables(file.params.front_tyre, built.params.front_tyre) &&
                       same_tables(file.params.rear_tyre, built.params.rear_tyre));
        const rigid_body_state &a = file.initial.body, &b = built.initial.body;
        check_true("same: start position and velocity", same_vec(a.pos, b.pos) && same_vec(a.vel, b.vel) &&
                                                            same_vec(a.omega, b.omega));
        check_true("same: start orientation", a.q.w == b.q.w && a.q.x == b.q.x && a.q.y == b.q.y && a.q.z == b.q.z);
        bool wheels_same = true;
        for (std::size_t i = 0; i < kWheels; ++i)
            wheels_same = wheels_same && file.initial.wheels[i].omega == built.initial.wheels[i].omega;
        check_true("same: wheels rolling", wheels_same);
    }

    // ---- [car] ------------------------------------------------------------------------------------------------
    {
        const four_wheel_run r = load(kMinimal + "[car]\nspring_front = 45 kN/m\nmass = 1.5 t\nbrake_bias_front = 62 %\n");
        check_near("car: a unit", r.params.front_corner.spring_rate, 45000, 0);
        check_near("car: tonnes", r.params.chassis.mass, 1500, 0);
        check_near("car: percent", r.params.drive_brake.brake_bias_front, 0.62, 1e-15);
        // a heavier car's tyres are refitted at their new loads
        check_near("car: tyres refitted", tmeasy_at(r.params.front_tyre.tmeasy_y, r.params.static_load(FL),
                                                    r.params.front_tyre.nominal_load).stiffness,
                   r.params.front_tyre.cornering_stiffness, 1e-6);
    }
    {
        const four_wheel_run both = load(kMinimal + "[car]\ntyre_curve = peak_slide\n");
        check_true("car: tyre_curve sets both", both.params.front_tyre.curve == tyre_curve::peak_slide &&
                                                   both.params.rear_tyre.curve == tyre_curve::peak_slide);
        const four_wheel_run split = load(kMinimal + "[car]\ntyre_curve_front = linear\n");
        check_true("car: one axle", split.params.front_tyre.curve == tyre_curve::linear &&
                                       split.params.rear_tyre.curve == tyre_curve::tmeasy);
        const four_wheel_run planar = load(kMinimal + "[car]\nvertical = planar\n");
        check_true("car: planar", planar.params.vertical == vertical_model::planar);
        check_near("car: planar starts on the ground plane", planar.initial.body.pos.z, 0, 0);
    }
    {
        // the default ride height follows the file's cg_height
        const four_wheel_run r = load(kMinimal + "[car]\ncg_height = 520 mm\n");
        check_near("car: start at the file's ride height", r.initial.body.pos.z, 0.52, 0);
    }

    // ---- [run] ------------------------------------------------------------------------------------------------
    {
        const four_wheel_run r = load("[run]\nduration = 10 s\ndt = 0.5 ms\n");
        check_near("run: dt", r.dt, 0.0005, 0);
        check_true("run: steps from duration / dt", r.steps == 20000);
        check_true("run: steps with the default dt", load("[run]\nduration = 2500 ms\n").steps == 2500);
    }

    // ---- [start] ----------------------------------------------------------------------------------------------
    {
        const four_wheel_run r = load(kMinimal + "[start]\nposition = (10, -5, 0.75)\nheading = 90 deg\nspeed = 20\n");
        check_near("start: position", r.initial.body.pos.x, 10, 0);
        check_near("start: in the air", r.initial.body.pos.z, 0.75, 0);
        check_near("start: heading", yaw_angle(r.initial.body.q), kPi / 2, 1e-12);
        check_near("start: speed along the heading (x)", r.initial.body.vel.x, 0, 1e-12);
        check_near("start: speed along the heading (y)", r.initial.body.vel.y, 20, 1e-12);
        check_near("start: wheels roll at the forward speed", r.initial.wheels[RL].omega * r.params.wheel.radius, 20,
                   1e-12);
    }
    {
        const four_wheel_run r = load(kMinimal + "[start]\nheading = 30 deg\npitch = -2 deg\nroll = 3 deg\n"
                                                 "velocity = (12, 7.5, 0) m/s\nangular_velocity = (0, -0.1, 0.2)\n");
        check_near("start: heading read back", yaw_angle(r.initial.body.q), 30 * kPi / 180, 1e-12);
        check_near("start: pitch read back (ISO: + nose down)", pitch_angle(r.initial.body.q), -2 * kPi / 180, 1e-12);
        check_near("start: roll read back (ISO: + right side down)", roll_angle(r.initial.body.q), 3 * kPi / 180, 1e-12);
        check_near("start: velocity, world", r.initial.body.vel.y, 7.5, 0);
        check_near("start: angular velocity, body", r.initial.body.omega.z, 0.2, 0);
        const Real forward = to_body(r.initial.body.q, r.initial.body.vel).x;
        check_near("start: wheels roll at the body-frame forward speed", r.initial.wheels[FL].omega,
                   forward / r.params.wheel.radius, 1e-12);
        check_true("start: sideways velocity is not rolling", forward < std::hypot(12.0, 7.5));
    }

    // ---- --set ------------------------------------------------------------------------------------------------
    {
        const std::string text = "[run]\nduration = 5 s\n[car]\nspring_front = 45 kN/m\n";
        const four_wheel_run r = load(text, {{"spring_front", "60000"}, {"run.dt", "0.5ms"}, {"start.speed", "10"}});
        check_near("--set: replaces a file value", r.params.front_corner.spring_rate, 60000, 0);
        check_near("--set: another section", r.dt, 0.0005, 0);
        check_true("--set: steps follow its dt", r.steps == 10000);
        check_near("--set: a section the file lacks", r.initial.body.vel.x, 10, 0);
        check_one_error("--set: an unknown key", kMinimal, "--set:", "sprng_front: unknown key in [car]",
                        {{"sprng_front", "1"}});
        check_one_error("--set: a bad value", kMinimal, "--set:", "mass: not a number", {{"mass", "abc"}});
    }

    // ---- no [driver]: the run still runs -----------------------------------------------------------------------
    {
        four_wheel_run r = load("[run]\nduration = 100 ms\n[start]\nspeed = 10\n");
        check_true("no driver: a script with one key", r.script.keys.size() == 1);
        const four_wheel_state end = run(r);
        check_true("no driver: 100 steps run", end.body.pos.x > 0.9 && end.body.pos.x < 1.01);
    }

    // ---- each error, on its line -------------------------------------------------------------------------------
    check_one_error("error: unknown section", kMinimal + "[track]\n", "t.ini:3:", "unknown section [track]");
    check_one_error("error: unknown key", kMinimal + "[start]\nsped = 2\n", "t.ini:4:", "sped: unknown key in [start]");
    check_one_error("error: [driver] not yet", kMinimal + "[driver]\nsteer = 0\n", "t.ini:3:", "[driver] is not supported yet");
    check_one_error("error: speed and velocity", kMinimal + "[start]\nspeed = 1\nvelocity = (1, 0, 0)\n", "t.ini:4:",
                    "give speed or velocity, not both");
    check_one_error("error: tyre_curve and one axle", kMinimal + "[car]\ntyre_curve = linear\ntyre_curve_rear = tmeasy\n",
                    "t.ini:4:", "not both");
    check_one_error("error: bad word", kMinimal + "[car]\nvertical = bouncy\n", "t.ini:4:", "expected one of springs, planar");
    check_one_error("error: other models", "[run]\nmodel = bicycle\nduration = 1 s\n", "t.ini:2:", "expected one of four_wheel");
    check_one_error("error: missing duration", "[run]\ndt = 1 ms\n", "t.ini:", "[run] duration is required");
    check_one_error("error: no file at all", "", "t.ini:", "[run] duration is required");
    check_one_error("error: duration not whole steps", "[run]\nduration = 1.0003 s\n", "t.ini:2:",
                    "not a whole number of 0.001 s steps");
    check_one_error("error: dt not above 0", "[run]\nduration = 1 s\ndt = 0\n", "t.ini:3:", "dt: must be above 0");
    check_one_error("error: duration not above 0", "[run]\nduration = -1 s\n", "t.ini:2:", "must be above 0");
    check_one_error("error: a bad value names its line", kMinimal + "[car]\nmass = 2\nspring_front = 50 km/h\n",
                    "t.ini:5:", "spring_front: unit km/h is not N/m");
    {
        // every problem at once, in line order
        const std::vector<std::string> e = errors_of("[run]\nduration = 1.0003 s\n[start]\nspeed = 1\nvelocity = (1,0,0)\n"
                                                     "sped = 2\n[car]\nspring_front = 50 km/h\n[track]\n");
        check_true("error: all five", e.size() == 5);
        int last = 0;
        bool ordered = true;
        for (const std::string &m : e)
        {
            const int line = std::stoi(m.substr(6)); // after "t.ini:"
            ordered = ordered && line >= last;
            last = line;
        }
        check_true("error: in line order", ordered);
    }
    {
        // malformed lines are layer 1's: they come as its ini_error, before any key is read
        check_one_error("error: a malformed line", "[run]\nduration 5\n", "t.ini:2:", "expected [section] or key = value");
    }

    // ---- the showcase file: every line reads; only [driver] is refused, until it is supported -----------------
    {
        const std::vector<std::string> e = [] {
            try
            {
                load_scenario_file(RALLYSIM_SOURCE_DIR "/scenarios/fw_showcase.ini", {});
            }
            catch (const ini_error &x)
            {
                return x.messages;
            }
            return std::vector<std::string>{};
        }();
        check_true("showcase: refused only for [driver]",
                   e.size() == 1 && e[0].find("[driver] is not supported yet") != std::string::npos);
    }

    return finish("scenario_file");
}
