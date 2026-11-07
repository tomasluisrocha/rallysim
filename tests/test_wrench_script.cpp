#include <cmath>
#include <cstdio>
#include "core/wrench_script.hpp"
#include "tests/check.hpp"

using namespace rallysim;

int main()
{
    wrench_script s;
    s.keys = {{1.0, wrench{vec3{10, 0, 0}, vec3{0, 0, 1}}},
              {3.0, wrench{vec3{30, 20, 0}, vec3{0, 0, 5}}},
              {4.0, wrench{vec3{30, 20, 0}, vec3{0, 0, 5}}}};

    check_true("valid script", is_valid(s));

    // before the first keyframe: hold the first value
    check_near("before first: force.x", sample(s, -5.0).force.x, 10.0);
    check_near("before first: torque.z", sample(s, 0.0).torque.z, 1.0);

    // exactly on keyframes: exact values, no rounding
    check_true("on first key", sample(s, 1.0).force.x == 10.0);
    check_true("on middle key", sample(s, 3.0).force.x == 30.0 && sample(s, 3.0).force.y == 20.0);

    // between keyframes: linear
    {
        const wrench w = sample(s, 2.0); // halfway between t=1 and t=3
        check_near("midway force.x", w.force.x, 20.0);
        check_near("midway force.y", w.force.y, 10.0);
        check_near("midway torque.z", w.torque.z, 3.0);
        const wrench q = sample(s, 1.5); // a quarter of the way
        check_near("quarter force.x", q.force.x, 15.0);
        check_near("quarter torque.z", q.torque.z, 2.0);
    }

    // flat segment stays flat
    check_near("flat segment force.x", sample(s, 3.5).force.x, 30.0);

    // after the last keyframe: hold the last value
    check_near("after last: force.y", sample(s, 100.0).force.y, 20.0);

    // a single keyframe is constant everywhere
    {
        wrench_script one;
        one.keys = {{2.0, wrench{vec3{0, 0, 7}, vec3{}}}};
        check_true("single key valid", is_valid(one));
        check_near("single key before", sample(one, 0.0).force.z, 7.0);
        check_near("single key after", sample(one, 9.0).force.z, 7.0);
    }

    // validity checks
    {
        wrench_script empty;
        check_true("empty script is invalid", !is_valid(empty));
        wrench_script bad;
        bad.keys = {{1.0, wrench{}}, {1.0, wrench{}}};
        check_true("equal times are invalid", !is_valid(bad));
        bad.keys = {{2.0, wrench{}}, {1.0, wrench{}}};
        check_true("decreasing times are invalid", !is_valid(bad));
    }

    return finish("wrench_script");
}