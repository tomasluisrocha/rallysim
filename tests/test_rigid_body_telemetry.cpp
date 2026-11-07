#include <cstdio>
#include <string>
#include <vector>
#include "core/rigid_body_telemetry.hpp"
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

// Checks that the value in the row under `name` is exactly `expected`.
static void check_column(const std::vector<std::string> &names, const std::vector<Real> &row,
                         const std::string &name, Real expected)
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
    if (row[static_cast<std::size_t>(i)] != expected)
    {
        std::printf("FAIL  column %s: got %.17g, expected %.17g\n", name.c_str(),
                    row[static_cast<std::size_t>(i)], expected);
        ++failures;
    }
}

int main()
{
    // Every field gets a different value, so any swap shows up.
    rigid_body_state s;
    s.pos = {1, 2, 3};
    s.vel = {4, 5, 6};
    s.q = {0.1, 0.2, 0.3, 0.4}; // not a valid rotation; only checking where the numbers land
    s.omega = {7, 8, 9};
    const wrench w{vec3{11, 12, 13}, vec3{14, 15, 16}};

    const std::vector<std::string> names = rigid_body_columns();
    const std::vector<std::string> units = rigid_body_units();
    const std::vector<Real> row = rigid_body_row(5, 0.001, s, w);

    // sizes
    check_true("21 column names", names.size() == 21);
    check_true("21 units", units.size() == 21);
    check_true("21 values in a row", row.size() == 21);

    // names are unique
    for (std::size_t i = 0; i < names.size(); ++i)
        for (std::size_t j = i + 1; j < names.size(); ++j)
            check_true("column names are unique", names[i] != names[j]);

    // each value lands under the right name
    check_column(names, row, "step", 5);
    check_column(names, row, "time", 5 * 0.001);
    check_column(names, row, "pos.x", 1);
    check_column(names, row, "pos.y", 2);
    check_column(names, row, "pos.z", 3);
    check_column(names, row, "vel.x", 4);
    check_column(names, row, "vel.y", 5);
    check_column(names, row, "vel.z", 6);
    check_column(names, row, "quat.w", 0.1);
    check_column(names, row, "quat.x", 0.2);
    check_column(names, row, "quat.y", 0.3);
    check_column(names, row, "quat.z", 0.4);
    check_column(names, row, "omega.x", 7);
    check_column(names, row, "omega.y", 8);
    check_column(names, row, "omega.z", 9);
    check_column(names, row, "force.x", 11);
    check_column(names, row, "force.y", 12);
    check_column(names, row, "force.z", 13);
    check_column(names, row, "torque.x", 14);
    check_column(names, row, "torque.y", 15);
    check_column(names, row, "torque.z", 16);

    // units match the columns they describe
    auto check_unit = [&](const char *name, const char *unit)
    {
        const int i = index_of(names, name);
        check_true(name, i >= 0 && units[static_cast<std::size_t>(i)] == unit);
    };
    check_unit("step", "count");
    check_unit("time", "s");
    check_unit("pos.z", "m");
    check_unit("vel.x", "m/s");
    check_unit("quat.w", "-");
    check_unit("omega.z", "rad/s");
    check_unit("force.y", "N");
    check_unit("torque.z", "N m");

    // the first two columns are step and time, as the schema says
    check_true("first column is step", names[0] == "step");
    check_true("second column is time", names[1] == "time");

    return finish("rigid_body_telemetry");
}
