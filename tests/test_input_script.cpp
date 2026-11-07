#include <cmath>
#include <cstdio>
#include "core/input_script.hpp"
#include "core/wrench_script.hpp"
#include "tests/check.hpp"

using namespace rallysim;

int main()
{
    input_script s;
    s.keys = {{1.0, bicycle_input{0.0, 1.0}},
              {3.0, bicycle_input{0.4, 3.0}},
              {4.0, bicycle_input{0.4, 3.0}}};

    check_true("valid script", is_valid(s));

    // before the first keyframe: hold the first value
    check_near("before first: steer", sample(s, -5.0).steer, 0.0);
    check_near("before first: accel", sample(s, 0.0).accel, 1.0);

    // exactly on keyframes: exact values, no rounding
    check_true("on first key", sample(s, 1.0).steer == 0.0 && sample(s, 1.0).accel == 1.0);
    check_true("on middle key", sample(s, 3.0).steer == 0.4 && sample(s, 3.0).accel == 3.0);

    // between keyframes: linear
    {
        const bicycle_input mid = sample(s, 2.0); // halfway between t=1 and t=3
        check_near("midway steer", mid.steer, 0.2);
        check_near("midway accel", mid.accel, 2.0);
        const bicycle_input quarter = sample(s, 1.5); // a quarter of the way
        check_near("quarter steer", quarter.steer, 0.1);
        check_near("quarter accel", quarter.accel, 1.5);
    }

    // flat segment stays flat
    check_near("flat segment steer", sample(s, 3.5).steer, 0.4);

    // after the last keyframe: hold the last value
    check_near("after last: accel", sample(s, 100.0).accel, 3.0);

    // a single keyframe is constant everywhere
    {
        input_script one;
        one.keys = {{2.0, bicycle_input{-0.3, 0.5}}};
        check_true("single key valid", is_valid(one));
        check_near("single key before", sample(one, 0.0).steer, -0.3);
        check_near("single key after", sample(one, 9.0).accel, 0.5);
    }

    // validity checks
    {
        input_script empty;
        check_true("empty script is invalid", !is_valid(empty));
        input_script bad;
        bad.keys = {{1.0, bicycle_input{}}, {1.0, bicycle_input{}}};
        check_true("equal times are invalid", !is_valid(bad));
        bad.keys = {{2.0, bicycle_input{}}, {1.0, bicycle_input{}}};
        check_true("decreasing times are invalid", !is_valid(bad));
    }

    // both script types in one file: the names must not clash
    {
        wrench_script w;
        w.keys = {{0.0, wrench{}}};
        check_true("wrench_script and input_script can be used together", is_valid(w) && is_valid(s));
    }

    return finish("input_script");
}
