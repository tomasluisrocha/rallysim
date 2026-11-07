#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "vehicle/bicycle_telemetry.hpp"
#include "tests/check.hpp"
#include "tests/telemetry_helpers.hpp"

using namespace rallysim;

int main()
{
    bicycle_params p; // lf = 1.3, lr = 1.2, max_steer = 0.6
    bicycle_state s;
    s.x = 1.5;
    s.y = -2.5;
    s.yaw = 0.75;
    s.v = 10.0;
    const bicycle_input in{0.2, 1.25};

    const std::vector<std::string> names = bicycle_columns();
    const std::vector<std::string> units = bicycle_units();
    const std::vector<Real> row = bicycle_row(5, 0.001, s, p, in);

    // sizes
    check_true("11 column names", names.size() == 11);
    check_true("11 units", units.size() == 11);
    check_true("11 values in a row", row.size() == 11);

    // names are unique
    for (std::size_t i = 0; i < names.size(); ++i)
        for (std::size_t j = i + 1; j < names.size(); ++j)
            check_true("column names are unique", names[i] != names[j]);

    // each value lands under the right name (every field has a different value, so a swap shows)
    check_column(names, row, "step", 5);
    check_column(names, row, "time", 5 * 0.001);
    check_column(names, row, "pos.x", 1.5);
    check_column(names, row, "pos.y", -2.5);
    check_column(names, row, "yaw", 0.75);
    check_column(names, row, "speed", 10.0);
    check_column(names, row, "steer", 0.2);
    check_column(names, row, "accel", 1.25);

    // derived values against closed forms written out here, not by calling the model's helpers
    const Real L = 2.5;
    const Real beta = std::atan(1.2 / L * std::tan(0.2));
    const Real rate = 10.0 * std::cos(beta) * std::tan(0.2) / L;
    check_column(names, row, "beta", beta, 1e-12);
    check_column(names, row, "yaw_rate", rate, 1e-12);
    check_column(names, row, "a_lat", 10.0 * rate, 1e-12);

    // the logged steer is the limited value: 3 rad is logged as max_steer, and the derived columns follow
    {
        const std::vector<Real> r = bicycle_row(0, 0.001, s, p, bicycle_input{3.0, 0.0});
        check_column(names, r, "steer", 0.6);
        check_column(names, r, "yaw_rate", 10.0 * std::cos(std::atan(1.2 / L * std::tan(0.6))) * std::tan(0.6) / L, 1e-12);
    }

    // sign: a right turn gives a negative yaw rate and a negative a_lat
    {
        const std::vector<Real> r = bicycle_row(0, 0.001, s, p, bicycle_input{-0.2, 0.0});
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
    check_unit("accel", "m/s^2");
    check_unit("beta", "rad");
    check_unit("yaw_rate", "rad/s");
    check_unit("a_lat", "m/s^2");

    // the first two columns are step and time, as the schema says
    check_true("first column is step", names[0] == "step");
    check_true("second column is time", names[1] == "time");

    return finish("bicycle_telemetry");
}
