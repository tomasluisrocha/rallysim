#pragma once
#include <cassert>
#include <cmath>

namespace rallysim
{
	using Real = double;
	inline constexpr Real kPi = 3.14159265358979323846;

	struct vec3
	{
		Real x = 0, y = 0, z = 0;

		Real &operator[](const int i)
		{
			assert(i >= 0 && i < 3);
			return i ? (1 == i ? y : z) : x;
		}
		Real operator[](const int i) const
		{
			assert(i >= 0 && i < 3);
			return i ? (1 == i ? y : z) : x;
		}
	};

	inline vec3 operator+(const vec3 &a, const vec3 &b)
	{
		return vec3{a.x + b.x, a.y + b.y, a.z + b.z};
	}

	inline vec3 &operator+=(vec3 &a, const vec3 &b)
	{
		a.x += b.x;
		a.y += b.y;
		a.z += b.z;
		return a;
	}

	inline vec3 operator-(const vec3 &a, const vec3 &b)
	{
		return vec3{a.x - b.x, a.y - b.y, a.z - b.z};
	}

	inline vec3 operator-(const vec3 &a)
	{
		return vec3{-a.x, -a.y, -a.z};
	}

	inline vec3 &operator-=(vec3 &a, const vec3 &b)
	{
		a.x -= b.x;
		a.y -= b.y;
		a.z -= b.z;
		return a;
	}

	inline vec3 operator*(const Real &scale, const vec3 &vector)
	{
		return vec3{scale * vector.x, scale * vector.y, scale * vector.z};
	}

	inline vec3 operator*(const vec3 &vector, const Real &scale)
	{
		return vec3{scale * vector.x, scale * vector.y, scale * vector.z};
	}

	inline vec3 operator/(const vec3 &vector, const Real &scale)
	{
		return vec3{vector.x / scale, vector.y / scale, vector.z / scale};
	}

	inline Real dot(const vec3 &a, const vec3 &b)
	{
		return (a.x * b.x + a.y * b.y + a.z * b.z);
	}

	inline Real norm(const vec3 &a)
	{
		return std::sqrt(dot(a, a));
	}

	inline vec3 normalized(const vec3 &a)
	{
		Real n = norm(a);
		// Guards against dividing by zero. Written so a NaN passes: a NaN state is reported by the runners'
		// finite guard (core/finite.hpp), which names the field and the step; an assert here could not.
		assert(!(n <= 0));
		return a / n;
	}

	inline vec3 cross(const vec3 &a, const vec3 &b)
	{
		return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
	}

	// True when every component is a real number: no NaN, no infinity (book ch. 26: fail on non-finite state).
	inline bool is_finite(const vec3 &v)
	{
		return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
	}

}
