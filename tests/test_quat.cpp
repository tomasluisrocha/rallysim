#include <cmath>
#include <cstdio>
#include "core/quat.hpp"

using namespace rallysim;

static int failures = 0;
static const Real tol = 1e-12;
static const Real pi = 3.14159265358979323846;

static void check_near(const char *what, Real actual, Real expected, Real t = tol)
{
    if (!(std::fabs(actual - expected) <= t))
    {
        std::printf("FAIL  %s: got %.12g, expected %.12g\n", what, actual, expected);
        ++failures;
    }
}

static void check_vec(const char *what, const vec3 &a, const vec3 &e, Real t = tol)
{
    if (!(std::fabs(a.x - e.x) <= t && std::fabs(a.y - e.y) <= t && std::fabs(a.z - e.z) <= t))
    {
        std::printf("FAIL  %s: got (%.12g, %.12g, %.12g), expected (%.12g, %.12g, %.12g)\n",
                    what, a.x, a.y, a.z, e.x, e.y, e.z);
        ++failures;
    }
}

static void check_quat(const char *what, const quat &a, const quat &e, Real t = tol)
{
    if (!(std::fabs(a.w - e.w) <= t && std::fabs(a.x - e.x) <= t &&
          std::fabs(a.y - e.y) <= t && std::fabs(a.z - e.z) <= t))
    {
        std::printf("FAIL  %s: got (%.12g, %.12g, %.12g, %.12g), expected (%.12g, %.12g, %.12g, %.12g)\n",
                    what, a.w, a.x, a.y, a.z, e.w, e.x, e.y, e.z);
        ++failures;
    }
}

// Rotates (1,0,0) with nsteps of integrate at 1 rad/s about z, for a total rotation of `angle`,
// and returns the error against the exact answer (cos, sin, 0).
static Real integrate_error(Real angle, int nsteps)
{
    const Real dt = angle / nsteps;
    quat q;
    for (int i = 0; i < nsteps; ++i)
        q = integrate(q, vec3{0, 0, 1}, dt);
    const vec3 r = rotate(q, vec3{1, 0, 0});
    return norm(r - vec3{std::cos(angle), std::sin(angle), 0});
}

int main()
{
    const quat id;
    const quat q1 = from_axis_angle(vec3{1, 2, 3}, 0.7); // an arbitrary unit rotation
    const quat qx90 = from_axis_angle(vec3{1, 0, 0}, pi / 2);
    const quat qy90 = from_axis_angle(vec3{0, 1, 0}, pi / 2);
    const quat qz90 = from_axis_angle(vec3{0, 0, 1}, pi / 2);
    const Real s = std::sqrt(0.5); // sin 45 deg = cos 45 deg

    // identity and default
    check_quat("default is identity", quat{}, quat{1, 0, 0, 0});
    check_quat("identity * q", id * q1, q1);
    check_quat("q * identity", q1 * id, q1);

    // conjugate, norm, normalized
    check_quat("conj", conj(quat{1, 2, 3, 4}), quat{1, -2, -3, -4});
    check_quat("q * conj(q) = identity", q1 * conj(q1), id);
    check_near("norm of identity", norm(id), 1.0);
    check_near("norm 1-2-2-4", norm(quat{1, 2, 2, 4}), 5.0);
    check_near("normalized has norm 1", norm(normalized(quat{1, 2, 2, 4})), 1.0);
    check_quat("normalized value", normalized(quat{0, 3, 0, 4}), quat{0, 0.6, 0, 0.8});

    // product: composition and order
    check_quat("two 90 deg z = one 180 deg z", qz90 * qz90, from_axis_angle(vec3{0, 0, 1}, pi));
    {
        const quat ab = qx90 * qy90, ba = qy90 * qx90;
        const bool same = std::fabs(ab.w - ba.w) < 1e-9 && std::fabs(ab.x - ba.x) < 1e-9 &&
                          std::fabs(ab.y - ba.y) < 1e-9 && std::fabs(ab.z - ba.z) < 1e-9;
        if (same)
        {
            std::printf("FAIL  product order: x90*y90 should differ from y90*x90\n");
            ++failures;
        }
    }

    // from_axis_angle
    check_quat("angle 0 = identity", from_axis_angle(vec3{0, 1, 0}, 0.0), id);
    check_quat("90 deg about z", qz90, quat{s, 0, 0, s});
    check_quat("180 deg about z", from_axis_angle(vec3{0, 0, 1}, pi), quat{0, 0, 0, 1});
    check_quat("non-unit axis same as unit axis", from_axis_angle(vec3{0, 0, 5}, pi / 2), qz90);
    check_near("from_axis_angle has norm 1", norm(from_axis_angle(vec3{3, -4, 12}, 1.234)), 1.0);

    // rotate: identity, the sign cases, and length preserved
    check_vec("identity rotate", rotate(id, vec3{1, 2, 3}), vec3{1, 2, 3});
    check_vec("z90: x -> y", rotate(qz90, vec3{1, 0, 0}), vec3{0, 1, 0});
    check_vec("z90: y -> -x", rotate(qz90, vec3{0, 1, 0}), vec3{-1, 0, 0});
    check_vec("x90: y -> z", rotate(qx90, vec3{0, 1, 0}), vec3{0, 0, 1});
    check_vec("x90: z -> -y", rotate(qx90, vec3{0, 0, 1}), vec3{0, -1, 0});
    check_vec("y90: z -> x", rotate(qy90, vec3{0, 0, 1}), vec3{1, 0, 0});
    check_vec("z90 leaves z axis alone", rotate(qz90, vec3{0, 0, 1}), vec3{0, 0, 1});
    check_near("rotate preserves length", norm(rotate(q1, vec3{1, 2, 3})), norm(vec3{1, 2, 3}));

    // rotate_inverse and composition
    {
        const vec3 v{1, -2, 3};
        check_vec("rotate_inverse undoes rotate", rotate_inverse(q1, rotate(q1, v)), v);
        check_vec("z90 inverse: y -> x", rotate_inverse(qz90, vec3{0, 1, 0}), vec3{1, 0, 0});
        check_vec("rotate(a*b) = rotate(a, rotate(b))", rotate(qx90 * qy90, v),
                  rotate(qx90, rotate(qy90, v)));
    }

    // integrate: 1000 steps of 1 ms at 1 rad/s about z.
    // Measured error is about 8e-8 (the renormalisation helps), so 1e-6 is the limit used here.
    {
        quat q;
        for (int i = 0; i < 1000; ++i)
            q = integrate(q, vec3{0, 0, 1}, 0.001);
        check_near("integrate keeps norm 1", norm(q), 1.0);
        check_vec("integrate: 1 rad about z", rotate(q, vec3{1, 0, 0}),
                  vec3{std::cos(1.0), std::sin(1.0), 0}, 1e-6);
    }
    // integrate converges: halving dt must clearly shrink the error
    {
        const Real e1 = integrate_error(1.0, 1000);
        const Real e2 = integrate_error(1.0, 2000);
        std::printf("integrate error: dt=1 ms %.3g, dt=0.5 ms %.3g\n", e1, e2);
        if (!(e2 < 0.6 * e1))
        {
            std::printf("FAIL  integrate: halving dt should shrink the error\n");
            ++failures;
        }
    }
    // integrate about a body axis other than z
    {
        quat q;
        for (int i = 0; i < 1000; ++i)
            q = integrate(q, vec3{1, 0, 0}, 0.001); // 1 rad/s about x
        check_vec("integrate about x: y -> (0, cos1, sin1)", rotate(q, vec3{0, 1, 0}),
                  vec3{0, std::cos(1.0), std::sin(1.0)}, 1e-6);
    }
    // integrate uses a BODY-frame angular velocity: starting from a rotated orientation,
    // spinning about the body x axis must give q0 * (rotation about x), not (rotation about x) * q0.
    {
        quat q = qz90;
        for (int i = 0; i < 1000; ++i)
            q = integrate(q, vec3{1, 0, 0}, 0.001);
        check_quat("integrate body frame: q0 * Rx(1)", q, qz90 * from_axis_angle(vec3{1, 0, 0}, 1.0), 1e-6);
    }

    if (failures)
        std::printf("%d check(s) failed\n", failures);
    else
        std::printf("all quat tests passed\n");
    return failures ? 1 : 0;
}