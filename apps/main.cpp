#include <cstdio>
#include <string>
#include "core/thrown_body.hpp"

using namespace rallysim;

int main(int argc, char **argv)
{
    const std::string path = (argc > 1) ? argv[1] : "run.csv";

    telemetry_writer writer(path, 0.001, rigid_body_columns(), rigid_body_units());
    if (!writer.ok())
    {
        std::printf("could not open %s for writing\n", path.c_str());
        return 1;
    }

    run_thrown_body(&writer);
    std::printf("wrote %s\n", path.c_str());
    return 0;
}