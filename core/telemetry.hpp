#pragma once
#include <cassert>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include "core/vec3.hpp"

namespace rallysim
{
    inline constexpr int kTelemetrySchema = 1;

    // Writes one CSV row per physics step. See docs/telemetry_schema.md for the format.
    class telemetry_writer
    {
    public:
        telemetry_writer(const std::string &path, Real dt,
                         const std::vector<std::string> &names,
                         const std::vector<std::string> &units)
            : columns_(names.size())
        {
            assert(names.size() == units.size());

            // Binary mode keeps "\n" as a single byte on every platform.
            out_.open(path, std::ios::binary);
            if (!out_.is_open())
                return;

            out_ << std::setprecision(17);

            out_ << "# schema=" << kTelemetrySchema << " dt=" << dt << " units:";
            for (std::size_t i = 0; i < names.size(); ++i)
                out_ << ' ' << names[i] << '[' << units[i] << ']';
            out_ << '\n';

            for (std::size_t i = 0; i < names.size(); ++i)
                out_ << (i ? "," : "") << names[i];
            out_ << '\n';
        }

        telemetry_writer(const telemetry_writer &) = delete;
        telemetry_writer &operator=(const telemetry_writer &) = delete;

        // False if the file could not be opened or a write has failed.
        bool ok() const { return out_.is_open() && out_.good(); }

        // Writes one row. Must have exactly one value per column.
        void row(const std::vector<Real> &values)
        {
            assert(values.size() == columns_);
            for (std::size_t i = 0; i < values.size(); ++i)
                out_ << (i ? "," : "") << values[i];
            out_ << '\n';
        }

    private:
        std::ofstream out_;
        std::size_t columns_;
    };
}