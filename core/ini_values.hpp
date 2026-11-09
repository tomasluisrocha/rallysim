#pragma once
#include <cmath>
#include <cstddef>
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
    inline constexpr unit_row units[] = {
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
    inline Real to_si(const quantity &q, const std::string &si);

    inline Real parse_number(const std::string &text, const std::string &si); // read_quantity + to_si
    inline vec3 parse_vector(const std::string &text, const std::string &si); // "(12, 7.5, 0) m/s"
    // "0 @ 0, 18 deg @ 3 s": the points in order; a single number is one point at t = 0, a constant
    inline keyframe_script<Real> parse_timeline(const std::string &text, const std::string &si);
    inline std::string parse_word(const std::string &text, const std::vector<std::string> &allowed);
}