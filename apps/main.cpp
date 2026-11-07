#include <cstdio>
#include <string>
#include "scenarios/bicycle_circle.hpp"
#include "scenarios/bicycle_lane_change.hpp"
#include "scenarios/bicycle_slalom.hpp"
#include "scenarios/driven_body.hpp"
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

int main(int argc, char **argv)
{
    const std::string name = (argc > 1) ? argv[1] : "thrown";
    const std::string path = (argc > 2) ? argv[2] : "run.csv";

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
    else
    {
        std::printf("usage: rallysim [thrown|driven|circle|lane|slalom] [output.csv]\n");
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