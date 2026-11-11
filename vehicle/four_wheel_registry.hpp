#pragma once
#include <stdexcept>
#include <string>
#include <vector>
#include "core/ini_values.hpp"
#include "vehicle/four_wheel.hpp"

namespace rallysim
{
    // When a parameter is used: some only matter to one vertical model or one tyre curve, and setting them on a car
    // that does not use them would change nothing (the scenario loader refuses that).
    enum class param_use
    {
        always,
        sprung,           // vertical = springs or unsprung: the corner springs and dampers
        unsprung,         // vertical = unsprung: the wheel mass, tyre spring and droop stop
        planar,           // vertical = planar: the quasi-static load transfer
        peak_slide_front, // the front tyre on the peak_slide curve: its shape
        peak_slide_rear,  // the rear tyre on the peak_slide curve
    };

    // The values a parameter may take. One outside is refused (range_error), never clamped.
    enum class param_range
    {
        any,
        positive,     // above 0
        non_negative, // 0 or more
        fraction,     // 0 to 1
        peak_shape,   // 1.5 to 3: the peak_slide curve's peak_slip
    };

    // A four-wheel parameter by name, so a run can be changed without rebuilding (rallysim --set, scenario files).
    // Every parameter is written on line 1 of the telemetry (four_wheel_meta), under the same name.
    struct param_entry
    {
        const char *name;
        const char *unit;
        Real &(*field)(four_wheel_params &); // where the value lives
        param_use use = param_use::always;
        param_range range = param_range::any;
    };

    // The tunable numbers. The tyres are fitted to the car when the params are built, so after setting mass, lf,
    // lr, a tyre stiffness (cx, calpha) or a nominal load (fz0), call rebuild_tyres() once, after all the settings.
    inline const std::vector<param_entry> &four_wheel_registry()
    {
        // PARAM("name", "unit", path inside four_wheel_params, range), PARAM_IF(..., path, use, range): the entry and
        // the lambda that reaches the field
#define PARAM_IF(name, unit, path, use, range) {name, unit, [](four_wheel_params &p) -> Real & { return p.path; }, use, range}
#define PARAM(name, unit, path, range) PARAM_IF(name, unit, path, param_use::always, range)
        static const std::vector<param_entry> table{
            // body
            PARAM("mass", "kg", chassis.mass, param_range::positive),
            PARAM("inertia_x", "kg m^2", chassis.inertia.x, param_range::positive),
            PARAM("inertia_y", "kg m^2", chassis.inertia.y, param_range::positive),
            PARAM("inertia_z", "kg m^2", chassis.inertia.z, param_range::positive),
            PARAM("lf", "m", lf, param_range::positive),
            PARAM("lr", "m", lr, param_range::positive),
            PARAM("track_front", "m", track_front, param_range::positive),
            PARAM("track_rear", "m", track_rear, param_range::positive),
            PARAM("cg_height", "m", cg_height, param_range::positive),
            PARAM_IF("front_transfer_share", "-", front_transfer_share, param_use::planar, param_range::fraction),
            PARAM("max_steer", "rad", max_steer, param_range::positive),
            // wheels
            PARAM("wheel_radius", "m", wheel.radius, param_range::positive),
            PARAM("wheel_inertia", "kg m^2", wheel.inertia, param_range::positive),
            // tyres: grip, stiffness and how fast the force builds up, on every curve
            PARAM("mu_front", "-", front_tyre.friction_coefficient, param_range::non_negative),
            PARAM("mu_rear", "-", rear_tyre.friction_coefficient, param_range::non_negative),
            PARAM("k_front", "-", front_tyre.load_sensitivity, param_range::non_negative),
            PARAM("k_rear", "-", rear_tyre.load_sensitivity, param_range::non_negative),
            PARAM("fz0_front", "N", front_tyre.nominal_load, param_range::positive),
            PARAM("fz0_rear", "N", rear_tyre.nominal_load, param_range::positive),
            PARAM("cx_front", "N", front_tyre.longitudinal_stiffness, param_range::positive),
            PARAM("cx_rear", "N", rear_tyre.longitudinal_stiffness, param_range::positive),
            PARAM("calpha_front", "N/rad", front_tyre.cornering_stiffness, param_range::positive),
            PARAM("calpha_rear", "N/rad", rear_tyre.cornering_stiffness, param_range::positive),
            PARAM("relaxation_x_front", "m", front_tyre.relaxation_length_x, param_range::positive),
            PARAM("relaxation_x_rear", "m", rear_tyre.relaxation_length_x, param_range::positive),
            PARAM("relaxation_y_front", "m", front_tyre.relaxation_length_y, param_range::positive),
            PARAM("relaxation_y_rear", "m", rear_tyre.relaxation_length_y, param_range::positive),
            // the peak_slide curve's shape (normalised slips; sliding force over peak force)
            PARAM_IF("peak_slip_front", "-", front_tyre.peak_slip, param_use::peak_slide_front, param_range::peak_shape),
            PARAM_IF("peak_slip_rear", "-", rear_tyre.peak_slip, param_use::peak_slide_rear, param_range::peak_shape),
            PARAM_IF("sliding_slip_front", "-", front_tyre.sliding_slip, param_use::peak_slide_front, param_range::positive),
            PARAM_IF("sliding_slip_rear", "-", rear_tyre.sliding_slip, param_use::peak_slide_rear, param_range::positive),
            PARAM_IF("sliding_ratio_front", "-", front_tyre.sliding_ratio, param_use::peak_slide_front, param_range::fraction),
            PARAM_IF("sliding_ratio_rear", "-", rear_tyre.sliding_ratio, param_use::peak_slide_rear, param_range::fraction),
            // suspension
            PARAM_IF("spring_front", "N/m", front_corner.spring_rate, param_use::sprung, param_range::positive),
            PARAM_IF("spring_rear", "N/m", rear_corner.spring_rate, param_use::sprung, param_range::positive),
            PARAM_IF("damper_front", "N s/m", front_corner.damper_rate, param_use::sprung, param_range::non_negative),
            PARAM_IF("damper_rear", "N s/m", rear_corner.damper_rate, param_use::sprung, param_range::non_negative),
            // the wheel and its strut (vertical = unsprung)
            PARAM_IF("unsprung_mass_front", "kg", front_corner.unsprung_mass, param_use::unsprung, param_range::positive),
            PARAM_IF("unsprung_mass_rear", "kg", rear_corner.unsprung_mass, param_use::unsprung, param_range::positive),
            PARAM_IF("tyre_rate_front", "N/m", front_corner.tyre_rate, param_use::unsprung, param_range::positive),
            PARAM_IF("tyre_rate_rear", "N/m", rear_corner.tyre_rate, param_use::unsprung, param_range::positive),
            PARAM_IF("tyre_damping_front", "N s/m", front_corner.tyre_damping, param_use::unsprung, param_range::non_negative),
            PARAM_IF("tyre_damping_rear", "N s/m", rear_corner.tyre_damping, param_use::unsprung, param_range::non_negative),
            PARAM_IF("droop_front", "m", front_corner.droop, param_use::unsprung, param_range::positive),
            PARAM_IF("droop_rear", "m", rear_corner.droop, param_use::unsprung, param_range::positive),
            PARAM_IF("rebound_ratio_front", "-", front_corner.rebound_ratio, param_use::unsprung, param_range::positive),
            PARAM_IF("rebound_ratio_rear", "-", rear_corner.rebound_ratio, param_use::unsprung, param_range::positive),
            PARAM_IF("bump_travel_front", "m", front_corner.bump_travel, param_use::unsprung, param_range::positive),
            PARAM_IF("bump_travel_rear", "m", rear_corner.bump_travel, param_use::unsprung, param_range::positive),
            PARAM_IF("bump_stop_length_front", "m", front_corner.bump_stop_length, param_use::unsprung, param_range::positive),
            PARAM_IF("bump_stop_length_rear", "m", rear_corner.bump_stop_length, param_use::unsprung, param_range::positive),
            PARAM_IF("bump_stop_rate_front", "N/m", front_corner.bump_stop_rate, param_use::unsprung, param_range::positive),
            PARAM_IF("bump_stop_rate_rear", "N/m", rear_corner.bump_stop_rate, param_use::unsprung, param_range::positive),
            PARAM_IF("droop_stop_rate_front", "N/m", front_corner.stop_rate, param_use::unsprung, param_range::positive),
            PARAM_IF("droop_stop_rate_rear", "N/m", rear_corner.stop_rate, param_use::unsprung, param_range::positive),
            // drive and brakes
            PARAM("max_drive_torque", "N m", drive_brake.max_drive_torque, param_range::non_negative),
            PARAM("max_drive_power", "W", drive_brake.max_drive_power, param_range::positive),
            PARAM("drive_split_front", "-", drive_brake.drive_split_front, param_range::fraction),
            PARAM("max_brake_torque", "N m", drive_brake.max_brake_torque, param_range::non_negative),
            PARAM("brake_bias_front", "-", drive_brake.brake_bias_front, param_range::fraction),
            PARAM("max_handbrake_torque", "N m", drive_brake.max_handbrake_torque, param_range::non_negative),
        };
#undef PARAM_IF
#undef PARAM
        return table;
    }

    // The entry called `name`, or nullptr if there is none.
    inline const param_entry *find_param(const std::string &name)
    {
        for (const param_entry &e : four_wheel_registry())
            if (name == e.name)
                return &e;
        return nullptr;
    }

    // Whether the car `p` uses the parameter at all.
    inline bool param_used(const param_entry &e, const four_wheel_params &p)
    {
        switch (e.use)
        {
        case param_use::sprung:
            return p.vertical != vertical_model::planar;
        case param_use::unsprung:
            return p.vertical == vertical_model::unsprung;
        case param_use::planar:
            return p.vertical == vertical_model::planar;
        case param_use::peak_slide_front:
            return p.front_tyre.curve == tyre_curve::peak_slide;
        case param_use::peak_slide_rear:
            return p.rear_tyre.curve == tyre_curve::peak_slide;
        default:
            return true;
        }
    }

    // When the parameter is used, for messages: "with vertical = unsprung"; empty for always.
    inline std::string param_use_text(const param_entry &e)
    {
        switch (e.use)
        {
        case param_use::sprung:
            return "with vertical = springs or unsprung";
        case param_use::unsprung:
            return "with vertical = unsprung";
        case param_use::planar:
            return "with vertical = planar";
        case param_use::peak_slide_front:
            return "with the front tyre on peak_slide";
        case param_use::peak_slide_rear:
            return "with the rear tyre on peak_slide";
        default:
            return "";
        }
    }

    // What is wrong with `value` for the parameter, or "" if it is within the parameter's range.
    inline std::string range_error(const param_entry &e, Real value)
    {
        switch (e.range)
        {
        case param_range::positive:
            return value > 0 ? "" : "must be above 0";
        case param_range::non_negative:
            return value >= 0 ? "" : "must be 0 or more";
        case param_range::fraction:
            return value >= 0 && value <= 1 ? "" : "must be within 0 to 1";
        case param_range::peak_shape:
            return value >= 1.5 && value <= 3 ? "" : "must be within 1.5 to 3";
        default:
            return "";
        }
    }

    // Sets the parameter `name` to `value`, given as text (from the command line or a file), read as a scenario-file
    // number: optionally with a unit of the same kind, converted to the parameter's own ("50 kN/m", "62 %").
    // Throws std::invalid_argument for an unknown name, a value that is not one finite number with such a unit
    // ("6e4abc" is refused, not read as 60000), or one outside the parameter's range; the car is then unchanged.
    inline void apply(four_wheel_params &p, const std::string &name, const std::string &value)
    {
        const param_entry *e = find_param(name);
        if (!e)
            throw std::invalid_argument("unknown parameter: " + name);
        Real v = 0;
        try
        {
            v = parse_number(value, e->unit);
        }
        catch (const std::invalid_argument &err)
        {
            throw std::invalid_argument("parameter " + name + ": " + err.what());
        }
        if (const std::string wrong = range_error(*e, v); !wrong.empty())
            throw std::invalid_argument("parameter " + name + ": " + wrong);
        e->field(p) = v;
    }
}
