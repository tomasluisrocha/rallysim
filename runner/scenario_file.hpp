#pragma once
#include <algorithm>
#include <cmath>
#include <fstream>
#include <istream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "core/ini.hpp"
#include "core/ini_values.hpp"
#include "runner/run_four_wheel.hpp"
#include "vehicle/four_wheel_registry.hpp"

namespace rallysim
{
    // Layer 3 of the scenario files: an .ini file's sections and keys turned into a four_wheel_run. The lines come
    // from parse_ini (core/ini.hpp), each value from core/ini_values.hpp; this layer knows what every key means.
    //
    // Sections and keys:
    //   [scenario]  name, description (text, for people)
    //   [run]       model (four_wheel), duration (s, required), dt (s, default 0.001)
    //   [start]     position (m), heading, pitch, roll (rad), speed (m/s) or velocity (m/s), angular_velocity (rad/s)
    //   [car]       every registry name (rallysim params), tyre_curve or tyre_curve_front / _rear, vertical
    //   [driver]    steer (rad), throttle, brake, handbrake (0 to 1) as timelines; hold_speed (m/s, a timeline) for
    //               the speed controller, in place of throttle and brake; hold_heading (rad) from hold_heading_from
    //               (s) for the heading-hold driver, with its gains heading_gain (1/s), max_yaw_rate (rad/s) and
    //               yaw_rate_gain (s). The drivers themselves: driver_input in run_four_wheel.hpp.

    // The keys a section takes, besides the registry's in [car]; empty for an unknown section.
    inline std::vector<std::string> scenario_section_keys(const std::string &section)
    {
        if (section == "scenario")
            return {"name", "description"};
        if (section == "run")
            return {"model", "duration", "dt"};
        if (section == "start")
            return {"position", "heading", "pitch", "roll", "speed", "velocity", "angular_velocity"};
        if (section == "car")
            return {"tyre_curve", "tyre_curve_front", "tyre_curve_rear", "vertical"};
        if (section == "driver")
            return {"steer", "throttle", "brake", "handbrake", "hold_speed", "hold_heading", "hold_heading_from",
                    "heading_gain", "max_yaw_rate", "yaw_rate_gain"};
        return {};
    }

    inline tyre_curve tyre_curve_named(const std::string &name)
    {
        return name == "linear" ? tyre_curve::linear : name == "peak_slide" ? tyre_curve::peak_slide : tyre_curve::tmeasy;
    }

    // Reads a scenario from `in` (`source` names it in messages). `overrides` are --set pairs, applied over the file:
    // "spring_front" is a [car] key, "run.dt" names its section. Throws ini_error with every problem in the file (and
    // in the overrides, shown as "--set"), each as "source:line: key: message".
    inline four_wheel_run load_scenario(std::istream &in, const std::string &source, const settings &overrides)
    {
        ini_file file = parse_ini(in, source); // throws on malformed lines

        // (line, message): sorted by line before they are thrown, so they read in file order. --set is line 0 (first),
        // a problem of the whole file (a missing key) comes last.
        constexpr int kWholeFile = 1 << 30;
        std::vector<std::pair<int, std::string>> errors;
        auto where = [&](int line) { return line > 0 ? source + ":" + std::to_string(line) : std::string("--set"); };
        auto report = [&](int line, const std::string &message)
        {
            errors.push_back({line, (line == kWholeFile ? source : where(line)) + ": " + message});
        };
        auto fail = [&](const ini_entry &e, const std::string &message) { report(e.line, e.key + ": " + message); };

        // --set replaces a file value or adds one; line 0 marks it as coming from the command line
        for (const auto &[name, value] : overrides)
        {
            const std::size_t dot = name.find('.');
            const std::string section_name = dot == std::string::npos ? "car" : name.substr(0, dot);
            const std::string key = dot == std::string::npos ? name : name.substr(dot + 1);
            auto section = std::find_if(file.sections.begin(), file.sections.end(),
                                        [&](const ini_section &s) { return s.name == section_name; });
            if (section == file.sections.end())
            {
                file.sections.push_back({section_name, 0, {}});
                section = file.sections.end() - 1;
            }
            auto entry = std::find_if(section->entries.begin(), section->entries.end(),
                                      [&](const ini_entry &e) { return e.key == key; });
            if (entry != section->entries.end())
                *entry = {key, value, 0};
            else
                section->entries.push_back({key, value, 0});
        }

        // every section and key must be known
        for (const ini_section &s : file.sections)
        {
            const std::vector<std::string> keys = scenario_section_keys(s.name);
            if (keys.empty())
            {
                report(s.line, "unknown section [" + s.name + "]");
                continue;
            }
            for (const ini_entry &e : s.entries)
            {
                const bool listed = std::find(keys.begin(), keys.end(), e.key) != keys.end();
                if (!listed && !(s.name == "car" && find_param(e.key)))
                    fail(e, "unknown key in [" + s.name + "]");
            }
        }

        auto entry_of = [&](const std::string &section, const std::string &key) -> const ini_entry *
        {
            for (const ini_section &s : file.sections)
                if (s.name == section)
                    for (const ini_entry &e : s.entries)
                        if (e.key == key)
                            return &e;
            return nullptr;
        };

        // Each reader stores the value and returns true, or records the error and returns false.
        auto number = [&](const ini_entry &e, const std::string &si, Real &out)
        {
            try
            {
                out = parse_number(e.value, si);
                return true;
            }
            catch (const std::invalid_argument &x)
            {
                fail(e, x.what());
                return false;
            }
        };
        auto vec = [&](const ini_entry &e, const std::string &si, vec3 &out)
        {
            try
            {
                out = parse_vector(e.value, si);
                return true;
            }
            catch (const std::invalid_argument &x)
            {
                fail(e, x.what());
                return false;
            }
        };
        auto word = [&](const ini_entry &e, const std::vector<std::string> &allowed, std::string &out)
        {
            try
            {
                out = parse_word(e.value, allowed);
                return true;
            }
            catch (const std::invalid_argument &x)
            {
                fail(e, x.what());
                return false;
            }
        };

        auto timeline = [&](const ini_entry &e, const std::string &si, keyframe_script<Real> &out)
        {
            try
            {
                out = parse_timeline(e.value, si);
                return true;
            }
            catch (const std::invalid_argument &x)
            {
                fail(e, x.what());
                return false;
            }
        };

        four_wheel_run run;

        // [car]: every number first, then the words, then the tyres fitted once to the finished car. Only if a number
        // the fit depends on was set (the weight on each wheel, or a stiffness): refitting a fitted tyre moves its
        // tables by a rounding step, and a car whose fit did not change must stay bit for bit the built-in one.
        const std::vector<std::string> fit_keys{"mass", "lf", "lr", "cx_front", "cx_rear", "calpha_front", "calpha_rear"};
        bool refit = false;
        for (const ini_section &s : file.sections)
            if (s.name == "car")
                for (const ini_entry &e : s.entries)
                    if (const param_entry *param = find_param(e.key))
                    {
                        Real v = 0;
                        if (number(e, param->unit, v))
                        {
                            param->field(run.params) = v;
                            refit = refit || std::find(fit_keys.begin(), fit_keys.end(), e.key) != fit_keys.end();
                        }
                    }
        const std::vector<std::string> curves{"linear", "peak_slide", "tmeasy"};
        std::string name;
        const ini_entry *both = entry_of("car", "tyre_curve");
        const ini_entry *front = entry_of("car", "tyre_curve_front");
        const ini_entry *rear = entry_of("car", "tyre_curve_rear");
        if (both && (front || rear))
            fail(*both, "give tyre_curve or tyre_curve_front and tyre_curve_rear, not both");
        else if (both && word(*both, curves, name))
            run.params.set_tyre_curve(tyre_curve_named(name));
        if (front && !both && word(*front, curves, name))
            run.params.front_tyre.curve = tyre_curve_named(name);
        if (rear && !both && word(*rear, curves, name))
            run.params.rear_tyre.curve = tyre_curve_named(name);
        if (const ini_entry *e = entry_of("car", "vertical"); e && word(*e, {"springs", "planar"}, name))
            run.params.vertical = name == "planar" ? vertical_model::planar : vertical_model::springs;
        if (refit)
            rebuild_tyres(run.params);

        // [run]
        if (const ini_entry *e = entry_of("run", "model"))
            word(*e, {"four_wheel"}, name);
        bool dt_ok = true;
        if (const ini_entry *e = entry_of("run", "dt"))
        {
            dt_ok = number(*e, "s", run.dt);
            if (dt_ok && !(run.dt > 0))
            {
                fail(*e, "must be above 0");
                dt_ok = false;
            }
        }
        Real duration = 0;
        if (const ini_entry *e = entry_of("run", "duration"))
        {
            if (number(*e, "s", duration) && dt_ok) // steps need a good dt (and duration / 0 must not reach llround)
            {
                const long long steps = std::llround(duration / run.dt);
                if (!(duration > 0))
                    fail(*e, "must be above 0");
                else if (std::fabs(static_cast<Real>(steps) * run.dt - duration) > 1e-9 * std::max(duration, Real(1)))
                    fail(*e, number_text(duration) + " s is not a whole number of " + number_text(run.dt) + " s steps");
                else
                    run.steps = static_cast<long>(steps);
            }
        }
        else
            report(kWholeFile, "[run] duration is required");

        // [start]: read after [car], whose cg_height and vertical model set the default ride height
        four_wheel_pose pose;
        pose.position = {0, 0, run.params.vertical == vertical_model::springs ? run.params.cg_height : 0};
        if (const ini_entry *e = entry_of("start", "position"))
            vec(*e, "m", pose.position);
        if (const ini_entry *e = entry_of("start", "heading"))
            number(*e, "rad", pose.heading);
        if (const ini_entry *e = entry_of("start", "pitch"))
            number(*e, "rad", pose.pitch);
        if (const ini_entry *e = entry_of("start", "roll"))
            number(*e, "rad", pose.roll);
        const ini_entry *speed = entry_of("start", "speed");
        const ini_entry *velocity = entry_of("start", "velocity");
        Real forward = 0;
        if (speed && velocity)
            fail(*speed, "give speed or velocity, not both");
        else if (speed && number(*speed, "m/s", forward))
            pose.velocity = {forward * std::cos(pose.heading), forward * std::sin(pose.heading), 0};
        else if (velocity)
            vec(*velocity, "m/s", pose.velocity);
        if (const ini_entry *e = entry_of("start", "angular_velocity"))
            vec(*e, "rad/s", pose.angular_velocity);
        run.initial = four_wheel_start(run.params, pose);

        // [driver]: the four inputs merged into one script. Each input is linear between its own points, so sampling
        // all four at every point time of any of them gives a script that is the same at every time. An input not
        // given is 0 throughout; with none given the script is one key of zeros (sample() needs a key).
        keyframe_script<Real> inputs[4] = {constant_script<Real>(0), constant_script<Real>(0), constant_script<Real>(0),
                                           constant_script<Real>(0)};
        const char *const input_names[4] = {"steer", "throttle", "brake", "handbrake"};
        std::vector<Real> times{0};
        for (int k = 0; k < 4; ++k)
            if (const ini_entry *e = entry_of("driver", input_names[k]);
                e && timeline(*e, k == 0 ? "rad" : "-", inputs[k]))
            {
                for (const keyframe<Real> &key : inputs[k].keys)
                {
                    times.push_back(key.t);
                    if (k > 0 && !(key.value >= 0 && key.value <= 1))
                        fail(*e, number_text(key.value) + " at " + number_text(key.t) + " s is outside 0 to 1");
                }
            }
        std::sort(times.begin(), times.end());
        times.erase(std::unique(times.begin(), times.end()), times.end());
        for (const Real t : times)
            run.script.keys.push_back({t, four_wheel_input{sample(inputs[0], t), sample(inputs[1], t),
                                                           sample(inputs[2], t), sample(inputs[3], t)}});

        if (const ini_entry *e = entry_of("driver", "hold_speed"))
        {
            if (entry_of("driver", "throttle") || entry_of("driver", "brake"))
                fail(*e, "the speed controller works the throttle and brake: give hold_speed or them, not both");
            else if (timeline(*e, "m/s", run.hold_speed))
                for (const keyframe<Real> &key : run.hold_speed.keys)
                    if (key.value < 0)
                        fail(*e, number_text(key.value) + " m/s at " + number_text(key.t) + " s is below 0");
        }

        const ini_entry *heading = entry_of("driver", "hold_heading");
        const ini_entry *heading_from = entry_of("driver", "hold_heading_from");
        if (heading && !heading_from)
            fail(*heading, "needs hold_heading_from, the time the heading-hold driver takes over");
        else if (heading_from && !heading)
            fail(*heading_from, "needs hold_heading, the heading to hold");
        else if (heading && heading_from)
        {
            number(*heading, "rad", run.hold_heading);
            if (number(*heading_from, "s", run.hold_heading_from) && run.hold_heading_from < 0)
                fail(*heading_from, "must be 0 or later");
        }
        struct gain
        {
            const char *key, *unit;
            Real &value;
        };
        for (const gain &g : {gain{"heading_gain", "1/s", run.heading_hold.heading_gain},
                              gain{"max_yaw_rate", "rad/s", run.heading_hold.max_yaw_rate},
                              gain{"yaw_rate_gain", "s", run.heading_hold.yaw_rate_gain}})
            if (const ini_entry *e = entry_of("driver", g.key); e && number(*e, g.unit, g.value) && !(g.value > 0))
                fail(*e, "must be above 0");

        if (!errors.empty())
        {
            std::stable_sort(errors.begin(), errors.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
            std::vector<std::string> messages;
            for (auto &e : errors)
                messages.push_back(std::move(e.second));
            throw ini_error(std::move(messages));
        }
        return run;
    }

    // The scenario in the file at `path`; see load_scenario. Throws std::runtime_error if the file cannot be opened.
    inline four_wheel_run load_scenario_file(const std::string &path, const settings &overrides)
    {
        std::ifstream in(path, std::ios::binary);
        if (!in)
            throw std::runtime_error("cannot open " + path);
        return load_scenario(in, path, overrides);
    }

    // The scenario files that come with the project: scenarios/<name>.ini, found through the folder CMake passes in
    // (RALLYSIM_SCENARIO_DIR), so rallysim and the tests find them from any working folder.
    inline std::string scenario_path(const std::string &name)
    {
        return std::string(RALLYSIM_SCENARIO_DIR) + "/" + name + ".ini";
    }

    // One of those files as a run: four_wheel_scenario("fw_brake"). Throws as load_scenario_file.
    inline four_wheel_run four_wheel_scenario(const std::string &name, const settings &overrides = {})
    {
        return load_scenario_file(scenario_path(name), overrides);
    }
}
