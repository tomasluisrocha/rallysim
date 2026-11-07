#pragma once
#include <cassert>
#include <cmath>
#include "core/vec3.hpp"

namespace rallysim
{
    struct quat
    {
        Real w = 1, x = 0, y = 0, z = 0;

        Real &operator[](const int i)
        {
            assert(i >= 0 && i < 4);
            switch (i)
            {
            case 0:
                return w;
            case 1:
                return x;
            case 2:
                return y;
            default:
                return z;
            }
        }
        Real operator[](const int i) const
        {
            assert(i >= 0 && i < 4);
            switch (i)
            {
            case 0:
                return w;
            case 1:
                return x;
            case 2:
                return y;
            default:
                return z;
            }
        }
    };

    inline quat operator*(const quat &a, const quat &b)
    {
        return quat{a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
    }

    inline quat conj(const quat &a)
    {
        return quat{a.w, -a.x, -a.y, -a.z};
    }

    inline Real norm(const quat &a)
    {
        return std::sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
    }

    inline quat normalized(const quat &a)
    {
        Real n = norm(a);
        assert(n > 0);
        return quat{a.w / n, a.x / n, a.y / n, a.z / n};
    }

    inline quat from_axis_angle(const vec3 &axis, Real angle)
    {
        const Real half = 0.5 * angle;
        const vec3 v = normalized(axis) * std::sin(half);
        return quat{std::cos(half), v.x, v.y, v.z};
    }

    inline vec3 to_world(const quat &q, const vec3 &v)
    {
        const vec3 u{q.x, q.y, q.z};
        const vec3 uv = cross(u, v);
        const vec3 uuv = cross(u, uv);
        return v + static_cast<Real>(2) * (q.w * uv + uuv);
    }

    inline vec3 to_body(const quat &q, const vec3 &v)
    {
        return to_world(conj(q), v);
    }

    inline quat integrate(const quat &q, const vec3 &omega_body, Real dt)
    {
        const quat omega_q{0, omega_body.x, omega_body.y, omega_body.z};
        const quat d = q * omega_q;
        const Real h = 0.5 * dt;
        return normalized(quat{q.w + h * d.w,
                               q.x + h * d.x,
                               q.y + h * d.y,
                               q.z + h * d.z});
    }
}
