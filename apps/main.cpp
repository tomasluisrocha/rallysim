#include <iostream>
#include "core/rigid_body.hpp"
#include "core/rigid_body_telemetry.hpp"
#include "core/telemetry.hpp"
#include <cstdio>
#include <string>

using namespace rallysim;

static wrench wrench_at(long i, const rigid_body_params &p)
{
	wrench w;
	if (i < 1000) // first second: cancel gravity, the body glides
		w.force = {0, 0, p.mass * kGravity};
	w.torque = {0, 0, 50.0}; // small constant torque about the body z axis
	return w;
}

static bool run_thrown_body(const std::string &path)
{
	const Real dt = 0.001;
	int steps = 3000;
	rigid_body_params p;
	p.mass = 1200;
	p.inertia = {500, 1800, 2000};
	rigid_body_state s;
	s.pos = {0, 0, 1};
	s.vel = {10, 0, 5};
	s.omega = {0, 0, 0.5};

	telemetry_writer writer(path, dt, rigid_body_columns(), rigid_body_units());
	if (!writer.ok())
	{
		return false;
	}

	for (long i = 0; i < steps; ++i)
	{
		writer.row(rigid_body_row(i, dt, s)); // log the state before advancing it
		step(s, p, wrench_at(i, p), dt);
	}

	writer.row(rigid_body_row(steps, dt, s)); // final state

	return writer.ok();
}

int main(int argc, char **argv)
{
	const std::string path = (argc > 1) ? argv[1] : "run.csv";
	if (!run_thrown_body(path))
	{
		std::printf("failed to write %s\n", path.c_str());
		return 1;
	}
	std::printf("wrote %s\n", path.c_str());
	return 0;
}
