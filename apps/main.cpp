#include <cstdio>
#include <filesystem>
#include <string>
#include <system_error>
#include "scenarios/bicycle_circle.hpp"
#include "scenarios/bicycle_lane_change.hpp"
#include "scenarios/bicycle_slalom.hpp"
#include "scenarios/driven_body.hpp"
#include "scenarios/single_track_circle.hpp"
#include "scenarios/single_track_lane_change.hpp"
#include "scenarios/thrown_body.hpp"

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
    telemetry_writer writer(path, scenario.dt, bicycle_columns(), bicycle_units());
    if (!writer.ok())
        return false;
    run(scenario, &writer);
    return true;
}

static bool write_single_track(const single_track_run &scenario, const std::string &path)
{
    telemetry_writer writer(path, scenario.dt, single_track_columns(), single_track_units());
    if (!writer.ok())
        return false;
    run(scenario, &writer);
    return true;
}

int main(int argc, char **argv)
{
    const std::string name = (argc > 1) ? argv[1] : "thrown";
    // Without a path the run goes to out/<scenario>.csv (out/ is ignored by git).
    const std::string path = (argc > 2) ? argv[2] : "out/" + name + ".csv";
    const std::filesystem::path folder = std::filesystem::path(path).parent_path();
    if (!folder.empty())
    {
        std::error_code ec;
        std::filesystem::create_directories(folder, ec); // if this fails, opening the file reports it
    }

    bool ok = false;
    if (name == "thrown")
        ok = write_rigid_body(thrown_body(), path);
    else if (name == "driven")
        ok = write_rigid_body(driven_body(), path);
    else if (name == "circle")
        ok = write_bicycle(bicycle_circle(), path);
    else if (name == "lane")
        ok = write_bicycle(bicycle_lane_change(), path);
    else if (name == "slalom")
        ok = write_bicycle(bicycle_slalom(), path);
    else if (name == "st_circle")
        ok = write_single_track(single_track_circle(), path);
    else if (name == "st_lane")
        ok = write_single_track(single_track_lane_change(), path);
    else
    {
        std::printf("usage: rallysim [thrown|driven|circle|lane|slalom|st_circle|st_lane] [output.csv]\n"
                    "  output defaults to out/<scenario>.csv\n");
        return 1;
    }

    if (!ok)
    {
        std::printf("could not open %s for writing\n", path.c_str());
        return 1;
    }
    std::printf("wrote %s (%s)\n", path.c_str(), name.c_str());
    return 0;
}