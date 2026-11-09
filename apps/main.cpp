#include <algorithm>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>
#include "scenarios/bicycle_circle.hpp"
#include "scenarios/bicycle_lane_change.hpp"
#include "scenarios/bicycle_slalom.hpp"
#include "scenarios/driven_body.hpp"
#include "scenarios/four_wheel_circle.hpp"
#include "scenarios/four_wheel_driving.hpp"
#include "scenarios/four_wheel_lane_change.hpp"
#include "scenarios/single_track_circle.hpp"
#include "scenarios/single_track_lane_change.hpp"
#include "scenarios/thrown_body.hpp"
#include "scenarios/tyre_rig.hpp"
#include "scenarios/run_four_wheel.hpp"
#include "scenarios/scenario_file.hpp"

using namespace rallysim;

static bool write_rigid_body(const scripted_run &scenario, const std::string &path)
{
    telemetry_writer writer(path, scenario.dt, rigid_body_columns(), rigid_body_units());
    if (!writer.ok())
        return false;
    run(scenario, &writer);
    return true;
}

static bool write_bicycle(const bicycle_run &scenario, const std::string &path)
{
    telemetry_writer writer(path, scenario.dt, bicycle_columns(), bicycle_units(), bicycle_meta(scenario.params));
    if (!writer.ok())
        return false;
    run(scenario, &writer);
    return true;
}

static bool write_single_track(const single_track_run &scenario, const std::string &path)
{
    telemetry_writer writer(path, scenario.dt, single_track_columns(), single_track_units(), single_track_meta(scenario.params));
    if (!writer.ok())
        return false;
    run(scenario, &writer);
    return true;
}

static bool write_four_wheel(const four_wheel_run &scenario, const std::string &path)
{
    telemetry_writer writer(path, scenario.dt, four_wheel_columns(), four_wheel_units(), four_wheel_meta(scenario.params));
    if (!writer.ok())
        return false;
    run(scenario, &writer);
    return true;
}

// Every scenario the app can write, with the folder under out/ its file goes to when no path is given: runs (one
// telemetry file per scenario) go to out/runs, the tyre rig (force curves, not a run) to out/rig. See docs/tools.md.
// A four-wheel scenario gives its run unwritten (four_wheel), so --set and `params` can reach its car; the others
// write themselves (write) and take no settings.
struct scenario
{
    const char *name;
    const char *folder;
    std::function<bool(const std::string &path)> write; // empty for a four-wheel scenario
    four_wheel_run (*four_wheel)() = nullptr;            // empty for the others
};

static const std::vector<scenario> &scenarios()
{
    static const std::vector<scenario> list{
        {"thrown", "runs", [](const std::string &p) { return write_rigid_body(thrown_body(), p); }},
        {"driven", "runs", [](const std::string &p) { return write_rigid_body(driven_body(), p); }},
        {"circle", "runs", [](const std::string &p) { return write_bicycle(bicycle_circle(), p); }},
        {"lane", "runs", [](const std::string &p) { return write_bicycle(bicycle_lane_change(), p); }},
        {"slalom", "runs", [](const std::string &p) { return write_bicycle(bicycle_slalom(), p); }},
        {"st_circle", "runs", [](const std::string &p) { return write_single_track(single_track_circle(), p); }},
        {"st_lane", "runs", [](const std::string &p) { return write_single_track(single_track_lane_change(), p); }},
        {"fw_circle", "runs", nullptr, four_wheel_circle},
        {"fw_lane", "runs", nullptr, four_wheel_lane_change},
        {"fw_brake", "runs", nullptr, four_wheel_brake},
        {"fw_launch", "runs", nullptr, four_wheel_launch},
        {"fw_handbrake", "runs", nullptr, four_wheel_handbrake},
        {"fw_hairpin", "runs", nullptr, four_wheel_hairpin},
        {"tyre_rig", "rig", [](const std::string &p) { return write_tyre_rig(p); }},
    };
    return list;
}

static const scenario *find_scenario(const std::string &name)
{
    const auto found = std::find_if(scenarios().begin(), scenarios().end(),
                                    [&](const scenario &s) { return name == s.name; });
    return found == scenarios().end() ? nullptr : &*found;
}

static void usage()
{
    std::fprintf(stderr,
                 "usage: rallysim <scenario | file.ini> [output.csv] [--set name=value]...\n"
                 "       rallysim list                         the built-in scenario names, one per line\n"
                 "       rallysim params [scenario | file.ini] what --set takes, with the car's values\n"
                 "  output defaults to out/runs/<scenario or file name>.csv (out/rig/ for tyre_rig)\n"
                 "  --set changes a four-wheel car: --set spring_front=50kN/m; in a file, also any key of\n"
                 "        another section: --set run.dt=0.5ms\n"
                 "  scenarios:");
    for (const scenario &s : scenarios())
        std::fprintf(stderr, " %s", s.name);
    std::fprintf(stderr, "\n");
}

// Every registry entry with its value in `p`.
static void print_params(four_wheel_params p)
{
    std::printf("%-22s %14s  %s\n", "name", "value", "unit");
    for (const param_entry &e : four_wheel_registry())
        std::printf("%-22s %14g  %s\n", e.name, e.field(p), e.unit);
}

static bool is_scenario_file(const std::string &name)
{
    return name.size() > 4 && name.compare(name.size() - 4, 4, ".ini") == 0;
}

// The four-wheel run of a scenario file or a built-in scenario, with the settings applied. Prints why and returns
// false if it cannot be built (a bad file, a bad setting, an unknown name).
static bool four_wheel_of(const std::string &name, const settings &set, four_wheel_run &out)
{
    try
    {
        if (is_scenario_file(name))
        {
            out = load_scenario_file(name, set);
            return true;
        }
        const scenario *s = find_scenario(name);
        if (!s || !s->four_wheel)
        {
            std::fprintf(stderr, "rallysim: '%s' is not a four-wheel scenario or an .ini file\n", name.c_str());
            return false;
        }
        out = with_settings(s->four_wheel(), set);
        return true;
    }
    catch (const ini_error &e)
    {
        std::fputs(e.what(), stderr); // every problem in the file, one per line
    }
    catch (const std::exception &e)
    {
        std::fprintf(stderr, "rallysim: %s\n", e.what());
    }
    return false;
}

int main(int argc, char **argv)
{
    const std::string name = (argc > 1) ? argv[1] : "thrown";
    if (name == "list")
    {
        for (const scenario &s : scenarios())
            std::printf("%s\n", s.name);
        return 0;
    }
    if (name == "params")
    {
        // the default car, or the car a four-wheel scenario or a file sets up
        if (argc < 3)
        {
            print_params(four_wheel_params{});
            return 0;
        }
        four_wheel_run r;
        if (!four_wheel_of(argv[2], {}, r))
            return 1;
        print_params(r.params);
        return 0;
    }
    const bool from_file = is_scenario_file(name);
    const scenario *found = from_file ? nullptr : find_scenario(name);
    if (!from_file && !found)
    {
        usage();
        return 1;
    }
    // the name for messages and the default output: the built-in name, or the file's name without .ini
    const std::string label = from_file ? std::filesystem::path(name).stem().string() : name;

    // the rest: an optional output path and any number of --set name=value
    std::string path;
    settings set;
    for (int k = 2; k < argc; ++k)
    {
        const std::string arg = argv[k];
        if (arg == "--set")
        {
            const char *pair = (k + 1 < argc) ? argv[k + 1] : nullptr;
            const char *eq = pair ? std::strchr(pair, '=') : nullptr;
            if (!eq)
            {
                std::fprintf(stderr, "rallysim: --set needs name=value\n");
                usage();
                return 1;
            }
            set.emplace_back(std::string(pair, eq), std::string(eq + 1));
            ++k;
        }
        else if (arg.rfind("--", 0) == 0)
        {
            std::fprintf(stderr, "rallysim: unknown option %s\n", arg.c_str());
            usage();
            return 1;
        }
        else if (path.empty())
            path = arg;
        else
        {
            std::fprintf(stderr, "rallysim: more than one output path (%s and %s)\n", path.c_str(), arg.c_str());
            return 1;
        }
    }
    const bool four_wheel = from_file || found->four_wheel;
    if (!set.empty() && !four_wheel)
    {
        std::fprintf(stderr, "rallysim: --set only works on the four-wheel scenarios, not %s\n", name.c_str());
        return 1;
    }

    // the run is built, settings and all, before any file is opened, so a bad one leaves no file behind
    std::function<bool(const std::string &)> write = four_wheel ? nullptr : found->write;
    if (four_wheel)
    {
        four_wheel_run r;
        if (!four_wheel_of(name, set, r))
            return 1;
        write = [r](const std::string &p) { return write_four_wheel(r, p); };
    }

    // Without a path the file goes to out/<folder>/<label>.csv (out/ is ignored by git).
    if (path.empty())
        path = std::string("out/") + (found ? found->folder : "runs") + "/" + label + ".csv";
    const std::filesystem::path folder = std::filesystem::path(path).parent_path();
    if (!folder.empty())
    {
        std::error_code ec;
        std::filesystem::create_directories(folder, ec); // if this fails, opening the file reports it
    }

    bool ok = false;
    try
    {
        ok = write(path);
    }
    catch (const std::exception &e)
    {
        // a non-finite state or telemetry value (core/finite.hpp): say where, and fail; the file stops there
        std::fprintf(stderr, "%s failed: %s\n", label.c_str(), e.what());
        return 1;
    }
    if (!ok)
    {
        std::fprintf(stderr, "could not open %s for writing\n", path.c_str());
        return 1;
    }
    std::printf("wrote %s (%s%s)\n", path.c_str(), label.c_str(), set.empty() ? "" : ", with --set");
    return 0;
}
