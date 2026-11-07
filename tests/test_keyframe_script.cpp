#include <cmath>
#include <cstdio>
#include "driver/scripts.hpp"
#include "tests/check.hpp"

using namespace rallysim;

// The keyframe template is tested once, on bicycle inputs, for timing, holding and validity. Each input
// type's lerp is then checked on its own, since that is the only part that differs between them.
int main()
{
    bicycle_script s;
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
        bicycle_script one;
        one.keys = {{2.0, bicycle_input{-0.3, 0.5}}};
        check_true("single key valid", is_valid(one));
        check_near("single key before", sample(one, 0.0).steer, -0.3);
        check_near("single key after", sample(one, 9.0).accel, 0.5);
    }

    // validity checks
    {
        bicycle_script empty;
        check_true("empty script is invalid", !is_valid(empty));
        bicycle_script bad;
        bad.keys = {{1.0, bicycle_input{}}, {1.0, bicycle_input{}}};
        check_true("equal times are invalid", !is_valid(bad));
        bad.keys = {{2.0, bicycle_input{}}, {1.0, bicycle_input{}}};
        check_true("decreasing times are invalid", !is_valid(bad));
    }

    // single-track inputs: steer and drive force both blend, exact on the keys
    {
        single_track_script st;
        st.keys = {{1.0, single_track_input{0.0, 1000.0}},
                   {3.0, single_track_input{0.4, 3000.0}}};
        const single_track_input mid = sample(st, 2.0);
        check_near("single track: midway steer", mid.steer, 0.2);
        check_near("single track: midway drive force", mid.drive_force, 2000.0);
        check_near("single track: quarter drive force", sample(st, 1.5).drive_force, 1500.0);
        check_true("single track: on a key, exact", sample(st, 3.0).steer == 0.4 && sample(st, 3.0).drive_force == 3000.0);
    }

    // wrenches: every component of force and torque blends
    {
        wrench_script w;
        w.keys = {{1.0, wrench{vec3{10, 0, 0}, vec3{0, 0, 1}}},
                  {3.0, wrench{vec3{30, 20, -4}, vec3{2, -2, 5}}}};
        const wrench mid = sample(w, 2.0);
        check_vec("wrench: midway force", mid.force, vec3{20, 10, -2});
        check_vec("wrench: midway torque", mid.torque, vec3{1, -1, 3});
        check_near("wrench: quarter torque.z", sample(w, 1.5).torque.z, 2.0);
        check_true("wrench: on a key, exact", sample(w, 3.0).force.y == 20.0 && sample(w, 3.0).torque.z == 5.0);
    }

    return finish("keyframe_script");
}
