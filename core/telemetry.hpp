#pragma once
#include <cassert>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "core/vec3.hpp"

namespace rallysim
{
    // 2: single-track axle and wheel columns renamed to dotted paths.
    // 3: four-wheel columns inserted (not appended): pos.z, roll, pitch after a_lat, and deflection, deflVel,
    //    springF, damperF at the end of each wheel's block.
    // 4: four-wheel body columns regrouped (time, inputs, where the car is, how it moves, then values worked out
    //    from those), with w, vel.z, roll_rate, pitch_rate and a_long added.
    // 5: four-wheel columns inserted: net.Fx, net.Fy and yaw_moment after a_lat, and grip after Fy in each wheel's
    //    block; line 1 lists every registry parameter.
    // 6: four-wheel columns inserted: tyreDefl, wheelZ and stopF after damperF in each wheel's block; line 1 gains
    //    vertical.
    inline constexpr int kTelemetrySchema = 6;

    // One column: its dotted name and its unit, written together so the two can never drift apart.
    struct telemetry_column
    {
        std::string name; // e.g. "wheel.FL.Fz"
        std::string unit; // e.g. "N"
    };

    inline std::vector<std::string> column_names(const std::vector<telemetry_column> &columns)
    {
        std::vector<std::string> names;
        for (const telemetry_column &c : columns)
            names.push_back(c.name);
        return names;
    }

    inline std::vector<std::string> column_units(const std::vector<telemetry_column> &columns)
    {
        std::vector<std::string> units;
        for (const telemetry_column &c : columns)
            units.push_back(c.unit);
        return units;
    }

    // Constant facts about the run that are not columns, written as key=value on line 1: the car's geometry (so a
    // viewer can draw it to scale) and every setting that shaped the run. Keys have no spaces and no '='.
    using telemetry_meta = std::vector<std::pair<std::string, Real>>;

    // Writes one CSV row per physics step. See docs/telemetry_schema.md for the format.
    class telemetry_writer
    {
    public:
        telemetry_writer(const std::string &path, Real dt, const std::vector<telemetry_column> &columns,
                         const telemetry_meta &meta = {})
            : columns_(columns.size()), names_(column_names(columns))
        {
            // Binary mode keeps "\n" as a single byte on every platform.
            out_.open(path, std::ios::binary);
            if (!out_.is_open())
                return;

            out_ << std::setprecision(17);

            out_ << "# schema=" << kTelemetrySchema << " dt=" << dt;
            for (const auto &kv : meta)
                out_ << ' ' << kv.first << '=' << kv.second;
            out_ << " units:";
            for (const telemetry_column &c : columns)
                out_ << ' ' << c.name << '[' << c.unit << ']';
            out_ << '\n';

            for (std::size_t i = 0; i < columns.size(); ++i)
                out_ << (i ? "," : "") << columns[i].name;
            out_ << '\n';
        }

        telemetry_writer(const telemetry_writer &) = delete;
        telemetry_writer &operator=(const telemetry_writer &) = delete;

        // False if the file could not be opened or a write has failed.
        bool ok() const { return out_.is_open() && out_.good(); }

        // Writes one row. Must have exactly one value per column. Every value must be finite: a NaN or infinity is
        // refused with an exception naming the column and the row, before anything of the row is written. Besides
        // marking a bug, MSVC would write it as "-nan(ind)", which the tools cannot read as a number (pandas turns
        // the whole column into text). The runners already stop on a non-finite state; this also covers the
        // columns worked out from it, such as forces and loads.
        void row(const std::vector<Real> &values)
        {
            assert(values.size() == columns_);
            for (std::size_t i = 0; i < values.size(); ++i)
                if (!std::isfinite(values[i]))
                    throw std::runtime_error("telemetry: column " + names_[i] + " is not finite in row " +
                                             std::to_string(rows_));
            for (std::size_t i = 0; i < values.size(); ++i)
                out_ << (i ? "," : "") << values[i];
            out_ << '\n';
            ++rows_;
        }

    private:
        std::ofstream out_;
        std::size_t columns_;
        std::vector<std::string> names_; // for naming a column in an error
        long rows_ = 0;                  // rows written so far
    };
}
