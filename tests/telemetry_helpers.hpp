#pragma once
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>
#include "tests/check.hpp"

// Helpers shared by the telemetry tests: find a column by name, as the plot tools do.

// Returns the index of a column name, or -1 if it is not there.
inline int index_of(const std::vector<std::string> &names, const std::string &name)
{
    for (std::size_t i = 0; i < names.size(); ++i)
        if (names[i] == name)
            return static_cast<int>(i);
    return -1;
}

// Checks the value in the row under `name` against `expected` within `tol` (0 = exactly).
inline void check_column(const std::vector<std::string> &names, const std::vector<rallysim::Real> &row,
                         const std::string &name, rallysim::Real expected, rallysim::Real tol = 0.0)
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
    const rallysim::Real got = row[static_cast<std::size_t>(i)];
    if (!(std::fabs(got - expected) <= tol))
    {
        std::printf("FAIL  column %s: got %.17g, expected %.17g\n", name.c_str(), got, expected);
        ++failures;
    }
}
