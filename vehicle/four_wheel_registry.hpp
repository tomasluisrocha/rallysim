#pragma once
#include <stdexcept>
#include <string>
#include <vector>
#include "core/ini_values.hpp"
#include "vehicle/four_wheel.hpp"

namespace rallysim
{
    // A four-wheel parameter by name, so a run can be changed without rebuilding (rallysim --set, scenario files).
    // Names match line 1 of the telemetry (four_wheel_meta) where both have the parameter.
    struct param_entry
    {
        const char *name;
        const char *unit;
        Real &(*field)(four_wheel_params &); // where the value lives
    };

    // The tunable numbers. The tyres are fitted to the car when the params are built, so after setting mass, lf,
    // lr or a tyre stiffness (cx, calpha), call rebuild_tyres() once, after all the settings.
    inline const std::vector<param_entry> &four_wheel_registry()
    {
        // PARAM("name", "unit", path inside four_wheel_params): the entry and the lambda that reaches the field
#define PARAM(name, unit, path) {name, unit, [](four_wheel_params &p) -> Real & { return p.path; }}
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
            PARAM("front_transfer_share", "-", front_transfer_share),
            PARAM("max_steer", "rad", max_steer),
            // wheels
            PARAM("wheel_radius", "m", wheel.radius),
            PARAM("wheel_inertia", "kg m^2", wheel.inertia),
            // tyres
            PARAM("mu_front", "-", front_tyre.friction_coefficient),
            PARAM("mu_rear", "-", rear_tyre.friction_coefficient),
            PARAM("k_front", "-", front_tyre.load_sensitivity),
            PARAM("k_rear", "-", rear_tyre.load_sensitivity),
            PARAM("cx_front", "N", front_tyre.longitudinal_stiffness),
            PARAM("cx_rear", "N", rear_tyre.longitudinal_stiffness),
            PARAM("calpha_front", "N/rad", front_tyre.cornering_stiffness),
            PARAM("calpha_rear", "N/rad", rear_tyre.cornering_stiffness),
            // suspension
            PARAM("spring_front", "N/m", front_corner.spring_rate),
            PARAM("spring_rear", "N/m", rear_corner.spring_rate),
            PARAM("damper_front", "N s/m", front_corner.damper_rate),
            PARAM("damper_rear", "N s/m", rear_corner.damper_rate),
            // drive and brakes
            PARAM("max_drive_torque", "N m", drive_brake.max_drive_torque),
            PARAM("max_drive_power", "W", drive_brake.max_drive_power),
            PARAM("drive_split_front", "-", drive_brake.drive_split_front),
            PARAM("max_brake_torque", "N m", drive_brake.max_brake_torque),
            PARAM("brake_bias_front", "-", drive_brake.brake_bias_front),
            PARAM("max_handbrake_torque", "N m", drive_brake.max_handbrake_torque),
        };
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
