#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "core/single_track_telemetry.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// Returns the index of a column name, or -1 if it is not there.
static int index_of(const std::vector<std::string> &names, const std::string &name)
{
    for (std::size_t i = 0; i < names.size(); ++i)
        if (names[i] == name)
            return static_cast<int>(i);
    return -1;
}

// Checks the value in the row under `name` against `expected` within `tol` (0 = exactly).
static void check_column(const std::vector<std::string> &names, const std::vector<Real> &row,
                         const std::string &name, Real expected, Real tol = 0.0)
{
    const int i = index_of(names, name);
    if (i < 0)
    {
        std::printf("FAIL  column %s is missing\n", name.c_str());
        ++failures;
        return;
    }
    if (static_cast<std::size_t>(i) >= row.size())
    {
        std::printf("FAIL  column %s: the row has only %zu values\n", name.c_str(), row.size());
        ++failures;
        return;
    }
    const Real got = row[static_cast<std::size_t>(i)];
    if (!(std::fabs(got - expected) <= tol))
    {
        std::printf("FAIL  column %s: got %.17g, expected %.17g\n", name.c_str(), got, expected);
        ++failures;
    }
}

// A car at (x, y) with heading `yaw`, velocity (u, v) in its own frame and yaw rate r.
static rigid_body_state car(Real x, Real y, Real yaw, Real u, Real v, Real r)
{
    rigid_body_state s;
    s.pos = vec3{x, y, 0};
    s.q = from_axis_angle(vec3{0, 0, 1}, yaw);
    s.vel = to_world(s.q, vec3{u, v, 0});
    s.omega = vec3{0, 0, r};
    return s;
}

int main()
{
    const single_track_params p; // 1200 kg, lf = 1.3, lr = 1.2, Cf = 60000, Cr = 70000, mu = 1, max_steer = 0.6
    const rigid_body_state s = car(1.5, -2.5, 0.75, 10.0, -0.3, 0.4);
    const single_track_input in{0.05, 800.0};

    const std::vector<std::string> names = single_track_columns();
    const std::vector<std::string> units = single_track_units();
    const std::vector<Real> row = single_track_row(5, 0.001, s, p, in);

    // sizes
    check_true("17 column names", names.size() == 17);
    check_true("17 units", units.size() == 17);
    check_true("17 values in a row", row.size() == 17);

    // names are unique
    for (std::size_t i = 0; i < names.size(); ++i)
        for (std::size_t j = i + 1; j < names.size(); ++j)
            check_true("column names are unique", names[i] != names[j]);

    // each value lands under the right name (every field has a different value, so a swap shows)
    check_column(names, row, "step", 5);
    check_column(names, row, "time", 5 * 0.001);
    check_column(names, row, "pos.x", 1.5, 1e-12);
    check_column(names, row, "pos.y", -2.5, 1e-12);
    check_column(names, row, "yaw", 0.75, 1e-12);
    check_column(names, row, "steer", 0.05);
    check_column(names, row, "drive_force", 800.0);
    check_column(names, row, "u", 10.0, 1e-12);
    check_column(names, row, "v", -0.3, 1e-12);
    check_column(names, row, "yaw_rate", 0.4, 1e-12);

    // derived values against closed forms written out here, not by calling the model's helpers
    const Real speed = std::hypot(10.0, -0.3);
    check_column(names, row, "speed", speed, 1e-12);
    check_column(names, row, "beta", std::atan2(-0.3, 10.0), 1e-12);
    check_column(names, row, "a_lat", speed * 0.4, 1e-12);

    const Real alpha_f = 0.05 - std::atan((-0.3 + 1.3 * 0.4) / 10.0);
    const Real alpha_r = -std::atan((-0.3 - 1.2 * 0.4) / 10.0);
    check_column(names, row, "alpha_f", alpha_f, 1e-12);
    check_column(names, row, "alpha_r", alpha_r, 1e-12);
    check_column(names, row, "fy_f", 60000.0 * alpha_f, 1e-6); // below the friction limit
    check_column(names, row, "fy_r", 70000.0 * alpha_r, 1e-6);

    // the logged steer is the limited value: 3 rad is logged as max_steer, and the slip follows it
    {
        const std::vector<Real> r = single_track_row(0, 0.001, s, p, single_track_input{3.0, 0.0});
        check_column(names, r, "steer", 0.6);
        check_column(names, r, "alpha_f", 0.6 - std::atan((-0.3 + 1.3 * 0.4) / 10.0), 1e-12);
        // with that much slip the front axle is at its friction limit: mu * W * lr / L
        check_column(names, r, "fy_f", 1.0 * 1200.0 * kGravity * 1.2 / 2.5, 1e-6);
    }

    // below the minimum forward speed, u is logged as it is; only the slip angles use the floor
    {
        const std::vector<Real> r = single_track_row(0, 0.001, car(0, 0, 0, 0.2, 0.1, 0.0), p, single_track_input{});
        check_column(names, r, "u", 0.2, 1e-12);
        check_column(names, r, "alpha_f", -std::atan(0.1 / kMinForwardSpeed), 1e-12);
    }

    // sign: a right turn gives a negative yaw rate and a negative a_lat
    {
        const std::vector<Real> r = single_track_row(0, 0.001, car(0, 0, 0, 10.0, 0.0, -0.4), p, single_track_input{-0.05, 0.0});
        check_true("right turn: yaw_rate < 0", r[static_cast<std::size_t>(index_of(names, "yaw_rate"))] < 0.0);
        check_true("right turn: a_lat < 0", r[static_cast<std::size_t>(index_of(names, "a_lat"))] < 0.0);
    }

    // units match the columns they describe
    auto check_unit = [&](const char *name, const char *unit)
    {
        const int i = index_of(names, name);
        check_true(name, i >= 0 && units[static_cast<std::size_t>(i)] == unit);
    };
    check_unit("step", "count");
    check_unit("time", "s");
    check_unit("pos.x", "m");
    check_unit("yaw", "rad");
    check_unit("speed", "m/s");
    check_unit("steer", "rad");
    check_unit("drive_force", "N");
    check_unit("u", "m/s");
    check_unit("v", "m/s");
    check_unit("yaw_rate", "rad/s");
    check_unit("beta", "rad");
    check_unit("a_lat", "m/s^2");
    check_unit("alpha_f", "rad");
    check_unit("alpha_r", "rad");
    check_unit("fy_f", "N");
    check_unit("fy_r", "N");

    // the first two columns are step and time, as the schema says
    check_true("first column is step", names[0] == "step");
    check_true("second column is time", names[1] == "time");

    // tools/topdown.py reads a file as a vehicle if it has these columns
    for (const char *needed : {"pos.x", "pos.y", "yaw", "steer", "speed", "a_lat"})
        check_true(needed, index_of(names, needed) >= 0);

    return finish("single_track_telemetry");
}