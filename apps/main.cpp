#include <cstdio>
#include <string>
#include "scenarios/driven_body.hpp"
#include "scenarios/thrown_body.hpp"

using namespace rallysim;

int main(int argc, char **argv)
{
	const std::string scenario = (argc > 1) ? argv[1] : "thrown";
	const std::string path = (argc > 2) ? argv[2] : "run.csv";

	if (scenario != "thrown" && scenario != "driven")
	{
		std::printf("usage: rallysim [thrown|driven] [output.csv]\n");
		return 1;
	}

	telemetry_writer writer(path, 0.001, rigid_body_columns(), rigid_body_units());
	if (!writer.ok())
	{
		std::printf("could not open %s for writing\n", path.c_str());
		return 1;
	}

	if (scenario == "thrown")
		run_thrown_body(&writer);
	else
		run_driven_body(&writer);

	std::printf("wrote %s (%s)\n", path.c_str(), scenario.c_str());
	return 0;
}