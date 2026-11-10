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
        springs,          // vertical = springs: the corner springs and dampers
        planar,           // vertical = planar: the quasi-static load transfer
        peak_slide_front, // the front tyre on the peak_slide curve: its shape
        peak_slide_rear,  // the rear tyre on the peak_slide curve
    };

    // A four-wheel parameter by name, so a run can be changed without rebuilding (rallysim --set, scenario files).
    // Every parameter is written on line 1 of the telemetry (four_wheel_meta), under the same name.
    struct param_entry
    {
        const char *name;
        const char *unit;
        Real &(*field)(four_wheel_params &); // where the value lives
        param_use use = param_use::always;
    };

    // The tunable numbers. The tyres are fitted to the car when the params are built, so after setting mass, lf,
    // lr, a tyre stiffness (cx, calpha) or a nominal load (fz0), call rebuild_tyres() once, after all the settings.
    inline const std::vector<param_entry> &four_wheel_registry()
    {
        // PARAM("name", "unit", path inside four_wheel_params[, use]): the entry and the lambda that reaches the field
#define PARAM(name, unit, path) {name, unit, [](four_wheel_params &p) -> Real & { return p.path; }}
#define PARAM_IF(name, unit, path, use) {name, unit, [](four_wheel_params &p) -> Real & { return p.path; }, use}
        static const std::vector<param_entry> table{
            // body
            PARAM("mass", "kg", chassis.mass),
            PARAM("inertia_x", "kg m^2", chassis.inertia.x),
            PARAM("inertia_y", "kg m^2", chassis.inertia.y),
            PARAM("inertia_z", "kg m^2", chassis.inertia.z),
            PARAM("lf", "m", lf),
            PARAM("lr", "m", lr),
            PARAM("track_front", "m", track_front),
            PARAM("track_rear", "m", track_rear),
            PARAM("cg_height", "m", cg_height),
            PARAM_IF("front_transfer_share", "-", front_transfer_share, param_use::planar),
            PARAM("max_steer", "rad", max_steer),
            // wheels
            PARAM("wheel_radius", "m", wheel.radius),
            PARAM("wheel_inertia", "kg m^2", wheel.inertia),
            // tyres: grip, stiffness and how fast the force builds up, on every curve
            PARAM("mu_front", "-", front_tyre.friction_coefficient),
            PARAM("mu_rear", "-", rear_tyre.friction_coefficient),
            PARAM("k_front", "-", front_tyre.load_sensitivity),
            PARAM("k_rear", "-", rear_tyre.load_sensitivity),
            PARAM("fz0_front", "N", front_tyre.nominal_load),
            PARAM("fz0_rear", "N", rear_tyre.nominal_load),
            PARAM("cx_front", "N", front_tyre.longitudinal_stiffness),
            PARAM("cx_rear", "N", rear_tyre.longitudinal_stiffness),
            PARAM("calpha_front", "N/rad", front_tyre.cornering_stiffness),
            PARAM("calpha_rear", "N/rad", rear_tyre.cornering_stiffness),
            PARAM("relaxation_x_front", "m", front_tyre.relaxation_length_x),
            PARAM("relaxation_x_rear", "m", rear_tyre.relaxation_length_x),
            PARAM("relaxation_y_front", "m", front_tyre.relaxation_length_y),
            PARAM("relaxation_y_rear", "m", rear_tyre.relaxation_length_y),
            // the peak_slide curve's shape (normalised slips; sliding force over peak force)
            PARAM_IF("peak_slip_front", "-", front_tyre.peak_slip, param_use::peak_slide_front),
            PARAM_IF("peak_slip_rear", "-", rear_tyre.peak_slip, param_use::peak_slide_rear),
            PARAM_IF("sliding_slip_front", "-", front_tyre.sliding_slip, param_use::peak_slide_front),
            PARAM_IF("sliding_slip_rear", "-", rear_tyre.sliding_slip, param_use::peak_slide_rear),
            PARAM_IF("sliding_ratio_front", "-", front_tyre.sliding_ratio, param_use::peak_slide_front),
            PARAM_IF("sliding_ratio_rear", "-", rear_tyre.sliding_ratio, param_use::peak_slide_rear),
            // suspension
            PARAM_IF("spring_front", "N/m", front_corner.spring_rate, param_use::springs),
            PARAM_IF("spring_rear", "N/m", rear_corner.spring_rate, param_use::springs),
            PARAM_IF("damper_front", "N s/m", front_corner.damper_rate, param_use::springs),
            PARAM_IF("damper_rear", "N s/m", rear_corner.damper_rate, param_use::springs),
            // drive and brakes
            PARAM("max_drive_torque", "N m", drive_brake.max_drive_torque),
            PARAM("max_drive_power", "W", drive_brake.max_drive_power),
            PARAM("drive_split_front", "-", drive_brake.drive_split_front),
            PARAM("max_brake_torque", "N m", drive_brake.max_brake_torque),
            PARAM("brake_bias_front", "-", drive_brake.brake_bias_front),
            PARAM("max_handbrake_torque", "N m", drive_brake.max_handbrake_torque),
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
        case param_use::springs:
            return p.vertical == vertical_model::springs;
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

    // When the parameter is used, for messages: "with vertical = springs"; empty for always.
    inline std::string param_use_text(const param_entry &e)
    {
        switch (e.use)
        {
        case param_use::springs:
            return "with vertical = springs";
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

    // Sets the parameter `name` to `value`, given as text (from the command line or a file), read as a scenario-file
    // number: optionally with a unit of the same kind, converted to the parameter's own ("50 kN/m", "62 %").
    // Throws std::invalid_argument for an unknown name, or a value that is not one finite number with such a unit
    // ("6e4abc" is refused, not read as 60000).
    inline void apply(four_wheel_params &p, const std::string &name, const std::string &value)
    {
        const param_entry *e = find_param(name);
        if (!e)
            throw std::invalid_argument("unknown parameter: " + name);
        try
        {
            e->field(p) = parse_number(value, e->unit);
        }
        catch (const std::invalid_argument &err)
        {
            throw std::invalid_argument("parameter " + name + ": " + err.what());
        }
    }
}
