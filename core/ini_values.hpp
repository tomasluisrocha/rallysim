#pragma once
#include <cmath>
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "core/ini.hpp" // trimmed()
#include "core/keyframe_script.hpp"
#include "core/vec3.hpp"

namespace rallysim
{
    // Layer 2 of the scenario files: one value's text to numbers. Knows nothing of keys or
    // files (it uses only trimmed() from layer 1); every function throws std::invalid_argument, and layer 3 adds the
    // file and line to the message.

    struct unit_row
    {
        const char *name; // Written unit
        const char *si;   // Canonical SI unit
        Real factor;      // Multiply input by this to get SI
    };

    // Every unit a value may be written in. The si strings are the registry's unit strings (four_wheel_registry.hpp).
    inline constexpr unit_row unit_table[] = {
        // Angle
        {"rad", "rad", 1},
        {"deg", "rad", kPi / 180},

        // Time
        {"s", "s", 1},
        {"ms", "s", 1e-3},

        // Length
        {"m", "m", 1},
        {"mm", "m", 1e-3},
        {"cm", "m", 1e-2},
        {"km", "m", 1e3},

        // Speed
        {"m/s", "m/s", 1},
        {"km/h", "m/s", 1.0 / 3.6},

        // Angular speed
        {"rad/s", "rad/s", 1},
        {"deg/s", "rad/s", kPi / 180},
        {"rpm", "rad/s", 2 * kPi / 60},

        // Mass
        {"kg", "kg", 1},
        {"t", "kg", 1e3},

        // Force
        {"N", "N", 1},
        {"kN", "N", 1e3},

        // Torque
        {"N m", "N m", 1},
        {"kN m", "N m", 1e3},

        // Power
        {"W", "W", 1},
        {"kW", "W", 1e3},
        {"hp", "W", 745.7},

        // Spring rate
        {"N/m", "N/m", 1},
        {"kN/m", "N/m", 1e3},
        {"N/mm", "N/m", 1e3},

        // Damper rate
        {"N s/m", "N s/m", 1},
        {"kN s/m", "N s/m", 1e3},

        // Cornering stiffness
        {"N/rad", "N/rad", 1},
        {"kN/rad", "N/rad", 1e3},

        // Inertia
        {"kg m^2", "kg m^2", 1},

        // Rate
        {"1/s", "1/s", 1},

        // Dimensionless ratio
        {"%", "-", 0.01},
    };

    // "50 km/h" -> {50, "km/h"};  "25deg" -> {25, "deg"};  "1200" -> {1200, ""}
    struct quantity
    {
        Real value;
        std::string unit; // empty when none was written
    };

    // '0' to '9' only: std::isdigit depends on the locale and is undefined for bytes above 127 (UTF-8 text).
    inline bool is_ascii_digit(char c) { return c >= '0' && c <= '9'; }

    // Index just past the number that starts at s[i], or i if there is none there. A number is
    // [+-] digits [. digits] [(e|E) [+-] digits], with at least one digit before the exponent (".5" and "5." count).
    // An exponent is taken only if a digit follows it, so "6e" is the number 6 then "e". No hex, inf or nan.
    inline std::size_t number_end(const std::string &s, std::size_t i)
    {
        auto digits_from = [&](std::size_t k)
        {
            while (k < s.size() && is_ascii_digit(s[k]))
                ++k;
            return k;
        };
        std::size_t k = i;
        if (k < s.size() && (s[k] == '+' || s[k] == '-'))
            ++k;
        std::size_t end = digits_from(k);
        bool any_digit = end > k;
        if (end < s.size() && s[end] == '.')
        {
            const std::size_t fraction = digits_from(end + 1);
            any_digit = any_digit || fraction > end + 1;
            end = fraction;
        }
        if (!any_digit)
            return i; // a sign or a '.' alone is not a number
        if (end < s.size() && (s[end] == 'e' || s[end] == 'E'))
        {
            std::size_t e = end + 1;
            if (e < s.size() && (s[e] == '+' || s[e] == '-'))
                ++e;
            const std::size_t exponent = digits_from(e);
            if (exponent > e)
                end = exponent;
        }
        return end;
    }

    // A value as written: a number, then optionally a unit. The number is found by number_end, so std::stod only
    // ever converts plain decimal text.
    inline quantity read_quantity(const std::string &text)
    {
        const std::string s = trimmed(text);
        if (s.empty())
            throw std::invalid_argument("missing value");

        const std::size_t end = number_end(s, 0);
        if (end == 0)
            throw std::invalid_argument("not a number: '" + s + "'");

        Real value = 0;
        try
        {
            value = std::stod(s.substr(0, end));
        }
        catch (const std::out_of_range &)
        {
            throw std::invalid_argument("number out of range: '" + s.substr(0, end) + "'");
        }
        if (!std::isfinite(value)) // cannot happen after the scan; kept as a guard
            throw std::invalid_argument("not a finite number: '" + s + "'");

        return {value, trimmed(s.substr(end))};
    }

    // The quantity in `si` (the key's own unit): 50 km/h with si "m/s" -> 13.888...
    inline Real to_si(const quantity &q, const std::string &si)
    {
        if (q.unit.empty())
            return q.value; // no unit written: already in the key's own unit

        for (const unit_row &u : unit_table)
        {
            if (q.unit == u.name)
            {
                if (si != u.si)
                    throw std::invalid_argument("unit " + q.unit + " is not " + si);
                return q.value * u.factor;
            }
        }
        throw std::invalid_argument("unknown unit '" + q.unit + "'");
    }

    inline Real parse_number(const std::string &text, const std::string &si) // read_quantity + to_si
    {
        return to_si(read_quantity(text), si);
    }

    // The pieces of `text` between each `separator`, untrimmed and in order; "a,,b" gives "a", "", "b", and text with
    // no separator is one piece.
    inline std::vector<std::string> split(const std::string &text, char separator)
    {
        std::vector<std::string> parts;
        for (std::size_t start = 0;;)
        {
            const std::size_t at = text.find(separator, start);
            parts.push_back(text.substr(start, at == std::string::npos ? std::string::npos : at - start));
            if (at == std::string::npos)
                return parts;
            start = at + 1;
        }
    }

    // A number for a message: 1.5, not 1.500000 (std::to_string's fixed six decimals).
    inline std::string number_text(Real v)
    {
        std::ostringstream out;
        out << v;
        return out.str();
    }

    // "(12, 7.5, 0) m/s": three numbers in brackets. A unit after the brackets applies to all three; a component may
    // carry its own unit instead ("(50 km/h, 0, 0)"), but not both. Exactly three parts.
    inline vec3 parse_vector(const std::string &text, const std::string &si)
    {
        const std::string s = trimmed(text);
        const std::size_t close = s.find(')');
        if (s.empty() || s.front() != '(' || close == std::string::npos)
            throw std::invalid_argument("expected a vector like (x, y, z), got '" + s + "'");

        const std::string inside = s.substr(1, close - 1);
        const std::string outer = trimmed(s.substr(close + 1)); // the unit for all three, or nothing
        if (inside.find('(') != std::string::npos || outer.find_first_of("()") != std::string::npos)
            throw std::invalid_argument("expected one vector like (x, y, z), got '" + s + "'");

        const std::vector<std::string> parts = split(inside, ',');
        if (parts.size() != 3)
            throw std::invalid_argument("a vector needs 3 numbers, got " + std::to_string(parts.size()));

        Real v[3];
        for (std::size_t k = 0; k < 3; ++k)
        {
            try
            {
                quantity q = read_quantity(parts[k]);
                if (!q.unit.empty() && !outer.empty())
                    throw std::invalid_argument("a unit both inside and after the brackets");
                if (q.unit.empty())
                    q.unit = outer;
                v[k] = to_si(q, si);
            }
            catch (const std::invalid_argument &e)
            {
                throw std::invalid_argument("component " + std::to_string(k + 1) + ": " + e.what());
            }
        }
        return {v[0], v[1], v[2]};
    }

    // "0 @ 0, 18 deg @ 3 s": points `value @ time`, the value in `si` and the time in s, times from 0 and strictly
    // increasing. Linear between points and held before the first and after the last (keyframe_script's sample).
    // A single number with no '@' is a constant: one point at t = 0.
    inline keyframe_script<Real> parse_timeline(const std::string &text, const std::string &si)
    {
        const std::string s = trimmed(text);
        if (s.empty())
            throw std::invalid_argument("missing value");

        keyframe_script<Real> script;
        if (s.find('@') == std::string::npos)
        {
            if (s.find(',') != std::string::npos)
                throw std::invalid_argument("expected value @ time points, got '" + s + "'");
            script.keys.push_back({0, parse_number(s, si)});
            return script;
        }

        const std::vector<std::string> points = split(s, ',');
        for (std::size_t k = 0; k < points.size(); ++k)
        {
            const std::string where = "point " + std::to_string(k + 1) + ": ";
            const std::vector<std::string> sides = split(points[k], '@');
            if (sides.size() != 2)
                throw std::invalid_argument(where + "expected value @ time, got '" + trimmed(points[k]) + "'");

            Real value = 0, time = 0;
            try
            {
                value = parse_number(sides[0], si);
                time = parse_number(sides[1], "s");
            }
            catch (const std::invalid_argument &e)
            {
                throw std::invalid_argument(where + e.what());
            }
            if (time < 0)
                throw std::invalid_argument(where + "time " + number_text(time) + " s is before the start");
            if (!script.keys.empty() && !(time > script.keys.back().t))
                throw std::invalid_argument(where + "time " + number_text(time) + " s is not after " +
                                            number_text(script.keys.back().t) + " s");
            script.keys.push_back({time, value});
        }
        return script;
    }

    // A name from a fixed list (tyre_curve = tmeasy, vertical = springs), checked and returned as written; the caller
    // turns it into its enum.
    inline std::string parse_word(const std::string &text, const std::vector<std::string> &allowed)
    {
        const std::string s = trimmed(text);
        for (const std::string &a : allowed)
            if (s == a)
                return s;
        std::string choices;
        for (std::size_t k = 0; k < allowed.size(); ++k)
            choices += (k ? ", " : "") + allowed[k];
        throw std::invalid_argument("expected one of " + choices + "; got '" + s + "'");
    }
}