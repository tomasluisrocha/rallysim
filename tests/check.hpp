#pragma once
#include <cmath>
#include <cstdio>
#include "core/rigid_body.hpp"

// Shared helpers for the test programs. Include it once per test and end main() with finish().

#if defined(_MSC_VER) && defined(_DEBUG)
#include <crtdbg.h>
#include <cstdlib>
// In a Debug build on Windows, a failed assert or an abort() opens a dialog and waits for a click. On the CI runner
// nobody clicks, so the job would hang until GitHub's timeout. Send those messages to the console instead: the test
// prints the message and stops, and CTest reports it as failed. Runs once, before main().
inline const bool tests_never_open_dialogs = []
{
    _set_error_mode(_OUT_TO_STDERR);
    _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ASSERT, _CRTDBG_FILE_STDERR);
    _CrtSetReportMode(_CRT_ERROR, _CRTDBG_MODE_FILE);
    _CrtSetReportFile(_CRT_ERROR, _CRTDBG_FILE_STDERR);
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
    return true;
}();
#endif

inline int failures = 0;

inline void check_true(const char *what, bool ok)
{
    if (!ok)
    {
        std::printf("FAIL  %s\n", what);
        ++failures;
    }
}

// Compare two numbers with a tolerance (never use == on doubles, except to test determinism).
inline void check_near(const char *what, rallysim::Real actual, rallysim::Real expected, rallysim::Real t = 1e-12)
{
    if (!(std::fabs(actual - expected) <= t))
    {
        std::printf("FAIL  %s: got %.17g, expected %.17g\n", what, actual, expected);
        ++failures;
    }
}

inline void check_vec(const char *what, const rallysim::vec3 &a, const rallysim::vec3 &e, rallysim::Real t = 1e-12)
{
    if (!(std::fabs(a.x - e.x) <= t && std::fabs(a.y - e.y) <= t && std::fabs(a.z - e.z) <= t))
    {
        std::printf("FAIL  %s: got (%.17g, %.17g, %.17g), expected (%.17g, %.17g, %.17g)\n",
                    what, a.x, a.y, a.z, e.x, e.y, e.z);
        ++failures;
    }
}

inline void check_quat(const char *what, const rallysim::quat &a, const rallysim::quat &e, rallysim::Real t = 1e-12)
{
    if (!(std::fabs(a.w - e.w) <= t && std::fabs(a.x - e.x) <= t &&
          std::fabs(a.y - e.y) <= t && std::fabs(a.z - e.z) <= t))
    {
        std::printf("FAIL  %s: got (%.17g, %.17g, %.17g, %.17g), expected (%.17g, %.17g, %.17g, %.17g)\n",
                    what, a.w, a.x, a.y, a.z, e.w, e.x, e.y, e.z);
        ++failures;
    }
}

// Bit-for-bit equality of two states, for determinism tests.
inline bool identical(const rallysim::rigid_body_state &a, const rallysim::rigid_body_state &b)
{
    return a.pos.x == b.pos.x && a.pos.y == b.pos.y && a.pos.z == b.pos.z &&
           a.vel.x == b.vel.x && a.vel.y == b.vel.y && a.vel.z == b.vel.z &&
           a.q.w == b.q.w && a.q.x == b.q.x && a.q.y == b.q.y && a.q.z == b.q.z &&
           a.omega.x == b.omega.x && a.omega.y == b.omega.y && a.omega.z == b.omega.z;
}

// Prints the summary and returns the exit code for main().
inline int finish(const char *name)
{
    if (failures)
        std::printf("%d check(s) failed\n", failures);
    else
        std::printf("all %s tests passed\n", name);
    return failures ? 1 : 0;
}
