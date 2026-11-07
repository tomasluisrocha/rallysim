#include <cmath>
#include <cstdio>
#include "core/bicycle.hpp"
#include "core/input_script.hpp"
#include "core/single_track_script.hpp"
#include "core/wrench_script.hpp"
#include "tests/check.hpp"

using namespace rallysim;

int main()
{
    single_track_script s;
    s.keys = {{1.0, single_track_input{0.0, 1000.0}},
              {3.0, single_track_input{0.4, 3000.0}},
              {4.0, single_track_input{0.4, 3000.0}}};

    check_true("valid script", is_valid(s));

    // before the first keyframe: hold the first value
    check_near("before first: steer", sample(s, -5.0).steer, 0.0);
    check_near("before first: drive force", sample(s, 0.0).drive_force, 1000.0);

    // exactly on keyframes: exact values, no rounding
    check_true("on first key", sample(s, 1.0).steer == 0.0 && sample(s, 1.0).drive_force == 1000.0);
    check_true("on middle key", sample(s, 3.0).steer == 0.4 && sample(s, 3.0).drive_force == 3000.0);

    // between keyframes: linear
    {
        const single_track_input mid = sample(s, 2.0); // halfway between t=1 and t=3
        check_near("midway steer", mid.steer, 0.2);
        check_near("midway drive force", mid.drive_force, 2000.0);
        const single_track_input quarter = sample(s, 1.5); // a quarter of the way
        check_near("quarter steer", quarter.steer, 0.1);
        check_near("quarter drive force", quarter.drive_force, 1500.0);
    }

    // flat segment stays flat
    check_near("flat segment steer", sample(s, 3.5).steer, 0.4);

    // after the last keyframe: hold the last value
    check_near("after last: drive force", sample(s, 100.0).drive_force, 3000.0);

    // a single keyframe is constant everywhere
    {
        single_track_script one;
        one.keys = {{2.0, single_track_input{-0.3, 500.0}}};
        check_true("single key valid", is_valid(one));
        check_near("single key before", sample(one, 0.0).steer, -0.3);
        check_near("single key after", sample(one, 9.0).drive_force, 500.0);
    }

    // validity checks
    {
        single_track_script empty;
        check_true("empty script is invalid", !is_valid(empty));
        single_track_script bad;
        bad.keys = {{1.0, single_track_input{}}, {1.0, single_track_input{}}};
        check_true("equal times are invalid", !is_valid(bad));
        bad.keys = {{2.0, single_track_input{}}, {1.0, single_track_input{}}};
        check_true("decreasing times are invalid", !is_valid(bad));
    }

    // all three script types in one file: the names must not clash
    {
        wrench_script w;
        w.keys = {{0.0, wrench{}}};
        input_script b;
        b.keys = {{0.0, bicycle_input{}}};
        check_true("the three script types can be used together", is_valid(w) && is_valid(b) && is_valid(s));
    }

    return finish("single_track_script");
}