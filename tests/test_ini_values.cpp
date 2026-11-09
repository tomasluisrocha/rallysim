#include <cstdio>
#include <functional>
#include <iterator>
#include <string>
#include <vector>
#include "check.hpp"
#include "core/ini_values.hpp"
#include "vehicle/four_wheel_registry.hpp"

using namespace rallysim;

namespace
{
    // True if `parse` throws std::invalid_argument with `words` in its message.
    bool fails(const std::function<void()> &parse, const std::string &words)
    {
        try
        {
            parse();
        }
        catch (const std::invalid_argument &e)
        {
            const bool ok = std::string(e.what()).find(words) != std::string::npos;
            if (!ok)
                std::printf("  wrong message: %s\n", e.what());
            return ok;
        }
        return false;
    }

    bool number_fails(const std::string &text, const std::string &si, const std::string &words)
    {
        return fails([&] { parse_number(text, si); }, words);
    }

    bool vector_fails(const std::string &text, const std::string &si, const std::string &words)
    {
        return fails([&] { parse_vector(text, si); }, words);
    }

    bool timeline_fails(const std::string &text, const std::string &si, const std::string &words)
    {
        return fails([&] { parse_timeline(text, si); }, words);
    }

    void check_vector(const char *what, const vec3 &v, Real x, Real y, Real z, Real tol = 0)
    {
        check_near(what, v.x, x, tol);
        check_near(what, v.y, y, tol);
        check_near(what, v.z, z, tol);
    }
}

int main()
{
    // ---- units ------------------------------------------------------------------------------------------------
    // every row converts by its own factor (a typo in a factor shows up here)
    for (const unit_row &u : unit_table)
        check_near(u.name, parse_number(std::string("1 ") + u.name, u.si), u.factor, 0);

    // every unit the registry uses can be written with a unit (a ratio, "-", only as %)
    for (const param_entry &e : four_wheel_registry())
    {
        bool found = false;
        for (const unit_row &u : unit_table)
            found = found || std::string(e.unit) == u.si;
        check_true(e.name, found);
    }

    // unit names are unique, or to_si would take the first and hide the second
    for (std::size_t i = 0; i < std::size(unit_table); ++i)
        for (std::size_t j = i + 1; j < std::size(unit_table); ++j)
            check_true(unit_table[i].name, std::string(unit_table[i].name) != unit_table[j].name);

    // ---- numbers ----------------------------------------------------------------------------------------------
    check_near("number: plain", parse_number("1200", "N"), 1200, 0);
    check_near("number: negative", parse_number("-0.4", "rad"), -0.4, 0);
    check_near("number: exponent", parse_number("6e4", "N/m"), 60000, 0);
    check_near("number: capital exponent", parse_number("1E-3", "m"), 0.001, 0);
    check_near("number: leading point", parse_number(".5", "m"), 0.5, 0);
    check_near("number: trailing point", parse_number("5.", "m"), 5, 0);
    check_near("number: plus sign", parse_number("+3", "m"), 3, 0);
    check_near("number: spaces around", parse_number("  7  ", "m"), 7, 0);
    check_near("number: unit with no space", parse_number("25deg", "rad"), 25 * (kPi / 180), 0);
    check_near("number: km/h", parse_number("50 km/h", "m/s"), 50 * (1.0 / 3.6), 0);
    check_near("number: unit with a space in it", parse_number("3.2 kN m", "N m"), 3200, 0);
    check_near("number: percent", parse_number("62 %", "-"), 0.62, 1e-15);
    check_near("number: no unit on a ratio", parse_number("0.62", "-"), 0.62, 0);

    const quantity q = read_quantity("  50  km/h ");
    check_true("read_quantity: number and unit, trimmed", q.value == 50 && q.unit == "km/h");

    // refused, each with its own message
    check_true("number: empty", number_fails("", "m", "missing value"));
    check_true("number: only spaces", number_fails("   ", "m", "missing value"));
    check_true("number: text", number_fails("abc", "m", "not a number"));
    check_true("number: inf", number_fails("inf", "m", "not a number"));
    check_true("number: nan", number_fails("-nan", "m", "not a number"));
    check_true("number: sign alone", number_fails("- 5", "m", "not a number"));
    check_true("number: overflow", number_fails("1e999", "m", "out of range"));
    check_true("number: hex", number_fails("0x10", "m", "unknown unit 'x10'"));
    check_true("number: junk after", number_fails("6e4abc", "N", "unknown unit 'abc'"));
    check_true("number: two points", number_fails("1.2.3", "m", "unknown unit '.3'"));
    check_true("number: wrong kind", number_fails("50 km/h", "N/m", "unit km/h is not N/m"));
    check_true("number: percent of a length", number_fails("5 %", "m", "unit % is not m"));
    check_true("number: unit spacing must match", number_fails("3 kN  m", "N m", "unknown unit"));

    // ---- split ------------------------------------------------------------------------------------------------
    check_true("split: three", split("a,b,c", ',') == std::vector<std::string>{"a", "b", "c"});
    check_true("split: keeps empty pieces", split("a,,b", ',') == std::vector<std::string>{"a", "", "b"});
    check_true("split: no separator", split("abc", ',') == std::vector<std::string>{"abc"});
    check_true("split: ends with a separator", split("a,", ',') == std::vector<std::string>{"a", ""});
    check_true("split: untrimmed", split(" a , b", ',') == std::vector<std::string>{" a ", " b"});

    // ---- vectors ----------------------------------------------------------------------------------------------
    check_vector("vector: unit after", parse_vector("(12, 7.5, 0) m/s", "m/s"), 12, 7.5, 0);
    check_vector("vector: no unit", parse_vector("(12, 7.5, 0)", "m/s"), 12, 7.5, 0);
    check_vector("vector: unit inside", parse_vector("(50 km/h, 0, 0)", "m/s"), 50 * (1.0 / 3.6), 0, 0);
    check_vector("vector: spaces and mm", parse_vector(" ( 1 , 2 , 3 ) mm ", "m"), 0.001, 0.002, 0.003);
    check_vector("vector: negative", parse_vector("(-1, -2e-1, 3)", "m"), -1, -0.2, 3);

    check_true("vector: two parts", vector_fails("(1, 2)", "m", "needs 3 numbers, got 2"));
    check_true("vector: four parts", vector_fails("(1, 2, 3, 4)", "m", "needs 3 numbers, got 4"));
    check_true("vector: unit inside and after", vector_fails("(1 m, 2, 3) m", "m", "both inside and after"));
    check_true("vector: no brackets", vector_fails("1, 2, 3", "m", "expected a vector"));
    check_true("vector: unclosed", vector_fails("(1, 2, 3", "m", "expected a vector"));
    check_true("vector: two vectors", vector_fails("(1, 2, 3)(4, 5, 6)", "m", "one vector"));
    check_true("vector: nested", vector_fails("((1, 2, 3))", "m", "one vector"));
    check_true("vector: empty component", vector_fails("(1,,3)", "m", "component 2: missing value"));
    check_true("vector: bad component", vector_fails("(1, 2, x)", "m", "component 3"));
    check_true("vector: wrong kind", vector_fails("(50 km/h, 0, 0)", "N/m", "component 1: unit km/h is not N/m"));
    check_true("vector: unknown unit after", vector_fails("(1, 2, 3) abc", "m", "unknown unit 'abc'"));

    // ---- timelines --------------------------------------------------------------------------------------------
    {
        const keyframe_script<Real> s = parse_timeline("0 @ 0, 18 deg @ 3 s", "rad");
        check_true("timeline: two points", s.keys.size() == 2);
        check_near("timeline: first time", s.keys[0].t, 0, 0);
        check_near("timeline: second value in rad", s.keys[1].value, 18 * (kPi / 180), 0);
        check_near("timeline: second time", s.keys[1].t, 3, 0);
        // the run's own sampling: linear between, held after the last point
        check_near("timeline: halfway", sample(s, 1.5), 9 * (kPi / 180), 1e-15);
        check_near("timeline: held after", sample(s, 10), 18 * (kPi / 180), 0);
    }
    {
        const keyframe_script<Real> s = parse_timeline("0.2", "-");
        check_true("timeline: a constant is one point at 0", s.keys.size() == 1 && s.keys[0].t == 0 && s.keys[0].value == 0.2);
        check_near("timeline: a constant everywhere", sample(s, 7), 0.2, 0);
    }
    {
        const keyframe_script<Real> s = parse_timeline("30 km/h @ 2500 ms", "m/s");
        check_near("timeline: time unit", s.keys[0].t, 2.5, 0);
        check_near("timeline: value unit", s.keys[0].value, 30 * (1.0 / 3.6), 0);
    }
    {
        // as a continued line arrives from layer 1: joined by spaces
        const keyframe_script<Real> s = parse_timeline("0 @ 3.2 s, 100 % @ 3.25 s, 100 % @ 3.55 s, 0 @ 3.6 s", "-");
        check_true("timeline: four points", s.keys.size() == 4 && s.keys[3].t == 3.6);
        check_near("timeline: 100 % is 1", s.keys[1].value, 1, 1e-15);
    }

    check_true("timeline: empty", timeline_fails("", "-", "missing value"));
    check_true("timeline: out of order", timeline_fails("1 @ 2, 0 @ 1", "-", "point 2: time 1 s is not after 2 s"));
    check_true("timeline: same time twice", timeline_fails("1 @ 1, 2 @ 1", "-", "not after"));
    check_true("timeline: before the start", timeline_fails("1 @ -1", "-", "point 1: time -1 s is before the start"));
    check_true("timeline: a point without '@'", timeline_fails("1 @ 0, 2", "-", "point 2: expected value @ time"));
    check_true("timeline: two '@'", timeline_fails("1 @ 2 @ 3", "-", "point 1: expected value @ time"));
    check_true("timeline: trailing comma", timeline_fails("1 @ 0,", "-", "point 2: expected value @ time"));
    check_true("timeline: a list without '@'", timeline_fails("0, 1", "-", "expected value @ time points"));
    check_true("timeline: bad value", timeline_fails("x @ 0", "-", "point 1: not a number"));
    check_true("timeline: wrong time unit", timeline_fails("1 @ 2 m", "-", "point 1: unit m is not s"));
    check_true("timeline: wrong value unit", timeline_fails("1 m @ 2", "rad", "point 1: unit m is not rad"));

    // ---- words ------------------------------------------------------------------------------------------------
    const std::vector<std::string> curves{"linear", "peak_slide", "tmeasy"};
    check_true("word: one of the list", parse_word("tmeasy", curves) == "tmeasy");
    check_true("word: trimmed", parse_word("  peak_slide ", curves) == "peak_slide");
    check_true("word: a typo lists the choices",
               fails([&] { parse_word("tmeasey", curves); }, "expected one of linear, peak_slide, tmeasy; got 'tmeasey'"));
    check_true("word: case matters", fails([&] { parse_word("TMeasy", curves); }, "expected one of"));

    // ---- the registry reads its values the same way ----------------------------------------------------------
    {
        four_wheel_params p;
        apply(p, "spring_front", "50 kN/m");
        check_near("apply: a unit", p.front_corner.spring_rate, 50000, 0);
        apply(p, "brake_bias_front", "62 %");
        check_near("apply: a percent", p.drive_brake.brake_bias_front, 0.62, 1e-15);
        apply(p, "max_steer", "30deg");
        check_near("apply: degrees", p.max_steer, 30 * (kPi / 180), 0);
        check_true("apply: the name in front of the reason",
                   fails([&] { apply(p, "spring_front", "50 km/h"); }, "parameter spring_front: unit km/h is not N/m"));
    }

    return finish("ini_values");
}
