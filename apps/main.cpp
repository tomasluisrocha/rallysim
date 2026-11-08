#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <string>
#include <system_error>
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
// telemetry file per scenario) go to out/runs, the tyre rig (force curves, not a run) to out/rig. See tools/README.md.
struct scenario
{
    const char *name;
    const char *folder;
    std::function<bool(const std::string &)> write;
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
        {"fw_circle", "runs", [](const std::string &p) { return write_four_wheel(four_wheel_circle(), p); }},
        {"fw_lane", "runs", [](const std::string &p) { return write_four_wheel(four_wheel_lane_change(), p); }},
        {"fw_brake", "runs", [](const std::string &p) { return write_four_wheel(four_wheel_brake(), p); }},
        {"fw_launch", "runs", [](const std::string &p) { return write_four_wheel(four_wheel_launch(), p); }},
        {"fw_handbrake", "runs", [](const std::string &p) { return write_four_wheel(four_wheel_handbrake(), p); }},
        {"tyre_rig", "rig", [](const std::string &p) { return write_tyre_rig(p); }},
    };
    return list;
}

static void usage()
{
    std::printf("usage: rallysim <scenario> [output.csv]\n"
                "       rallysim list          (the scenario names, one per line)\n"
                "  output defaults to out/runs/<scenario>.csv (out/rig/ for tyre_rig)\n  scenarios:");
    for (const scenario &s : scenarios())
        std::printf(" %s", s.name);
    std::printf("\n");
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
    const auto found = std::find_if(scenarios().begin(), scenarios().end(),
                                    [&](const scenario &s) { return name == s.name; });
    if (found == scenarios().end())
    {
        usage();
        return 1;
    }

    // Without a path the file goes to out/<folder>/<scenario>.csv (out/ is ignored by git).
    const std::string path = (argc > 2) ? argv[2] : std::string("out/") + found->folder + "/" + name + ".csv";
    const std::filesystem::path folder = std::filesystem::path(path).parent_path();
    if (!folder.empty())
    {
        std::error_code ec;
        std::filesystem::create_directories(folder, ec); // if this fails, opening the file reports it
    }

    const bool ok = found->write(path);
    if (!ok)
    {
        std::printf("could not open %s for writing\n", path.c_str());
        return 1;
    }
    std::printf("wrote %s (%s)\n", path.c_str(), name.c_str());
    return 0;
}