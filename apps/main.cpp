#include <cstdio>
#include <string>
#include "scenarios/driven_body.hpp"
#include "scenarios/thrown_body.hpp"

using namespace rallysim;

int main(int argc, char **argv)
{
    const std::string name = (argc > 1) ? argv[1] : "thrown";
    const std::string path = (argc > 2) ? argv[2] : "run.csv";

    if (name != "thrown" && name != "driven")
    {
        std::printf("usage: rallysim [thrown|driven] [output.csv]\n");
        return 1;
    }
    const scripted_run scenario = (name == "thrown") ? thrown_body() : driven_body();

    telemetry_writer writer(path, scenario.dt, rigid_body_columns(), rigid_body_units());
    if (!writer.ok())
    {
        std::printf("could not open %s for writing\n", path.c_str());
        return 1;
    }

    run(scenario, &writer);
    std::printf("wrote %s (%s)\n", path.c_str(), name.c_str());
    return 0;
}