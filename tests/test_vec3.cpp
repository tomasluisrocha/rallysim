#include <cmath>
#include <cstdio>
#include "core/vec3.hpp"

using namespace rallysim;

static int failures = 0;
static const Real tol = 1e-12;

// Compare two numbers with a tolerance (never use == on doubles).
static void check_near(const char *what, Real actual, Real expected)
{
    if (!(std::fabs(actual - expected) <= tol))
    {
        std::printf("FAIL  %s: got %.12g, expected %.12g\n", what, actual, expected);
        ++failures;
    }
}

// Compare two vectors component by component.
static void check_vec(const char *what, const vec3 &actual, const vec3 &expected)
{
    if (!(std::fabs(actual.x - expected.x) <= tol &&
          std::fabs(actual.y - expected.y) <= tol &&
          std::fabs(actual.z - expected.z) <= tol))
    {
        std::printf("FAIL  %s: got (%.12g, %.12g, %.12g), expected (%.12g, %.12g, %.12g)\n",
                    what, actual.x, actual.y, actual.z, expected.x, expected.y, expected.z);
        ++failures;
    }
}

int main()
{
    // default construction is zero
    check_vec("default is zero", vec3{}, vec3{0, 0, 0});

    // + - and unary -
    check_vec("add", vec3{1, 2, 3} + vec3{4, 5, 6}, vec3{5, 7, 9});
    check_vec("subtract", vec3{5, 5, 5} - vec3{1, 2, 3}, vec3{4, 3, 2});
    check_vec("unary minus", -vec3{1, -2, 3}, vec3{-1, 2, -3});

    // scalar multiply, both orders, and divide
    check_vec("scalar * vec", 2.0 * vec3{1, 2, 3}, vec3{2, 4, 6});
    check_vec("vec * scalar", vec3{1, 2, 3} * 2.0, vec3{2, 4, 6});
    check_vec("vec / scalar", vec3{2, 4, 6} / 2.0, vec3{1, 2, 3});

    // dot, norm
    check_near("dot", dot(vec3{1, 2, 3}, vec3{4, 5, 6}), 32.0);
    check_near("dot of perpendicular vectors", dot(vec3{1, 0, 0}, vec3{0, 1, 0}), 0.0);
    check_near("norm 3-4-5", norm(vec3{3, 4, 0}), 5.0);
    check_near("norm of zero", norm(vec3{}), 0.0);

    // cross: right-handed, x forward, y left, z up
    check_vec("x cross y = z", cross(vec3{1, 0, 0}, vec3{0, 1, 0}), vec3{0, 0, 1});
    check_vec("y cross z = x", cross(vec3{0, 1, 0}, vec3{0, 0, 1}), vec3{1, 0, 0});
    check_vec("z cross x = y", cross(vec3{0, 0, 1}, vec3{1, 0, 0}), vec3{0, 1, 0});
    check_vec("y cross x = -z", cross(vec3{0, 1, 0}, vec3{1, 0, 0}), vec3{0, 0, -1});
    check_vec("v cross v = 0", cross(vec3{1, 2, 3}, vec3{1, 2, 3}), vec3{0, 0, 0});
    // cross result is perpendicular to both inputs
    {
        vec3 a{1, 2, 3}, b{-2, 0.5, 4};
        vec3 c = cross(a, b);
        check_near("cross perpendicular to a", dot(c, a), 0.0);
        check_near("cross perpendicular to b", dot(c, b), 0.0);
    }

    // += and -= modify the left side and can be chained
    {
        vec3 v{1, 2, 3};
        v += vec3{4, 5, 6};
        check_vec("+=", v, vec3{5, 7, 9});
        v -= vec3{1, 1, 1};
        check_vec("-=", v, vec3{4, 6, 8});
        vec3 w{1, 1, 1};
        (v += w) += w;
        check_vec("chained +=", v, vec3{6, 8, 10});
    }

    // indexing: non-const (read and write) and const
    {
        vec3 v{7, 8, 9};
        check_near("v[0]", v[0], 7);
        check_near("v[1]", v[1], 8);
        check_near("v[2]", v[2], 9);
        v[1] = 20;
        check_vec("write through []", v, vec3{7, 20, 9});
        const vec3 c{1, 2, 3};
        check_near("const v[2]", c[2], 3);
    }

    if (failures)
        std::printf("%d check(s) failed\n", failures);
    else
        std::printf("all vec3 tests passed\n");
    return failures ? 1 : 0;
}