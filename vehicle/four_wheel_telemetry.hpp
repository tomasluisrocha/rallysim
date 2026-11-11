#pragma once
#include <cmath>
#include <string>
#include <utility>
#include <vector>
#include "core/telemetry.hpp"
#include "vehicle/four_wheel.hpp"
#include "vehicle/four_wheel_energy.hpp"
#include "vehicle/four_wheel_registry.hpp"

namespace rallysim
{
    inline const char *wheel_label(std::size_t i)
    {
        static const char *const labels[kWheels] = {"FL", "FR", "RL", "RR"};
        return labels[i];
    }

    // The per-wheel columns, in this order for each wheel.
    inline std::vector<telemetry_column> wheel_fields()
    {
        return {{"omega", "rad/s"}, {"slipRatio", "-"}, {"slipAngle", "rad"}, {"Fz", "N"}, {"Fx", "N"}, {"Fy", "N"},
                {"grip", "N"}, {"driveTorque", "N m"}, {"brakeTorque", "N m"}, {"deflection", "m"}, {"deflVel", "m/s"},
                {"springF", "N"}, {"damperF", "N"}, {"tyreDefl", "m"}, {"wheelZ", "m"},
                {"stopF", "N"}};
    }

    // The energy books, after the wheels: what is stored, term by term, the work done since the start of the run by
    // kind, and the residual (vehicle/four_wheel_energy.hpp). All in J.
    inline std::vector<telemetry_column> energy_columns()
    {
        std::vector<telemetry_column> c;
        for (const char *name : {"energy.translation", "energy.rotation", "energy.gravity", "energy.springs",
                                 "energy.wheels", "energy.total", "work.drive", "work.brake", "work.damper", "work.tyre",
                                 "energy.residual"})
            c.push_back({name, "J"});
        return c;
    }

    // The body columns come in groups: time, the driver's inputs, where the car is (world position, then roll, pitch
    // and yaw), how it moves (body_velocity's u, v, w, p, q, r, in that order), then values worked out from those and
    // from the tyre forces. Then each wheel's block and the energy books.
    inline std::vector<telemetry_column> four_wheel_columns()
    {
        std::vector<telemetry_column> c{{"step", "count"}, {"time", "s"},
                                        {"steer", "rad"}, {"throttle", "0-1"}, {"brake", "0-1"}, {"handbrake", "0-1"},
                                        {"pos.x", "m"}, {"pos.y", "m"}, {"pos.z", "m"},
                                        {"roll", "rad"}, {"pitch", "rad"}, {"yaw", "rad"},
                                        {"u", "m/s"}, {"v", "m/s"}, {"w", "m/s"},
                                        {"roll_rate", "rad/s"}, {"pitch_rate", "rad/s"}, {"yaw_rate", "rad/s"},
                                        {"speed", "m/s"}, {"beta", "rad"}, {"vel.z", "m/s"},
                                        {"a_long", "m/s^2"}, {"a_lat", "m/s^2"},
                                        {"net.Fx", "N"}, {"net.Fy", "N"}, {"yaw_moment", "N m"}};
        for (std::size_t i = 0; i < kWheels; ++i)
            for (const telemetry_column &f : wheel_fields())
                c.push_back({std::string("wheel.") + wheel_label(i) + "." + f.name, f.unit});
        for (const telemetry_column &e : energy_columns())
            c.push_back(e);
        return c;
    }

    // A tyre's TMeasy tables for line 1, as tmeasy_<axis>_<field><1 or 2>_<axle>: 1 at the nominal load, 2 at twice
    // it (see tmeasy_point in tyre.hpp).
    inline void add_tmeasy_meta(telemetry_meta &meta, const tyre_params &t, const char *axle)
    {
        for (const auto &[axis, d] : {std::pair<const char *, const tmeasy_direction *>{"x", &t.tmeasy_x}, {"y", &t.tmeasy_y}})
            for (const auto &[n, point] : {std::pair<const char *, const tmeasy_point *>{"1", &d->nominal}, {"2", &d->double_load}})
            {
                const std::string at = std::string(n) + "_" + axle;
                const std::string base = std::string("tmeasy_") + axis + "_";
                meta.push_back({base + "stiffness" + at, point->stiffness});
                meta.push_back({base + "peak_slip" + at, point->peak_slip});
                meta.push_back({base + "sliding_slip" + at, point->sliding_slip});
                meta.push_back({base + "sliding_ratio" + at, point->sliding_ratio});
            }
    }

    // Line 1 of the file: the model and its choices (vertical 0 planar, 1 springs, 2 unsprung; springs 1 for both
    // sprung modes, 0 for planar; curve 0 linear, 1 peak_slide, 2 tmeasy), then
    // every parameter of the registry under its own name, so the file records the whole car that made it (rallysim
    // --set included), then each tyre's TMeasy tables. A viewer draws the car from it to scale.
    inline telemetry_meta four_wheel_meta(const four_wheel_params &p)
    {
        telemetry_meta meta{{"model", 4},
                            {"springs", p.vertical != vertical_model::planar ? 1.0 : 0.0},
                            {"vertical", static_cast<Real>(p.vertical)},
                            {"curve_front", static_cast<Real>(p.front_tyre.curve)},
                            {"curve_rear", static_cast<Real>(p.rear_tyre.curve)}};
        four_wheel_params copy = p; // the registry reaches fields through a non-const reference
        for (const param_entry &e : four_wheel_registry())
            meta.push_back({e.name, e.field(copy)});
        add_tmeasy_meta(meta, p.front_tyre, "front");
        add_tmeasy_meta(meta, p.rear_tyre, "rear");
        return meta;
    }

    // `input` is the command applied at this step (the pedals as given, the steer limited); the motion and the
    // forces are computed by evaluate(), the same function the step uses. pos.z is the CG's height; roll and pitch
    // follow ISO 8855 (positive: right side down, nose down). u, v, w and the three rates are body_velocity, in the
    // body's axes: w tilts with the body, so it is not heave; vel.z, the world vertical speed, is. a_long is the
    // force the step applies to the body, along the car's heading on the ground, over the mass; only the tyres push
    // along the ground (the springs and gravity act vertically), so it is their net push. a_lat is speed x yaw_rate,
    // the steady-turn form, so in fast transients the two are not worked out the same way. net.Fx and net.Fy are the
    // four tyre forces summed in the body's axes, and yaw_moment their moment about the CG (positive turning left):
    // what turns the car.
    //
    // slipRatio and slipAngle are the steady (kinematic) slips of the contact patch, from the wheel's spin and the
    // patch velocity: -1 is a locked wheel sliding forward. The relaxed slips inside the wheel are capped where the
    // force stops changing (slip_cap_scale), so they say how far the tyre is twisted, not how fast it slides. Fx and
    // Fy are in the wheel's frame; grip is the most force the tyre can take at that load, mu(Fz) Fz (tyre_grip);
    // driveTorque and brakeTorque are what the pedals put on that wheel. deflection, deflVel, springF and damperF
    // are the corner's suspension (zero in the planar model); in the unsprung model they are the strut's and Fz is
    // the tyre's. tyreDefl, wheelZ and stopF (the droop stop, zero or negative) are zero except in that model. The energy columns are this state's stored energy and
    // `books`' work and residual: the books must hold the work of every step before this one. See
    // docs/telemetry_schema.md.
    inline std::vector<Real> four_wheel_row(long step, Real dt, const four_wheel_state &s,
                                            const four_wheel_params &p, const four_wheel_input &input,
                                            const energy_books &books)
    {
        const four_wheel_forces f = evaluate(s, p, input);
        const Real speed = std::hypot(f.vel.u, f.vel.v);
        const Real yaw = yaw_angle(s.body.q);
        const vec3 force = four_wheel_wrench(s, p, f).force; // world frame
        const Real a_long = (force.x * std::cos(yaw) + force.y * std::sin(yaw)) / p.chassis.mass;
        vec3 net;            // the tyre forces summed, body frame
        Real yaw_moment = 0; // their moment about the CG, about the body's z axis
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            const contact_geometry c = contact_of(p, s.body, i, f.steer);
            const vec3 tyre = to_body(s.body.q, f.tyre[i].fx * c.forward + f.tyre[i].fy * c.left);
            net += tyre;
            yaw_moment += c.point.x * tyre.y - c.point.y * tyre.x;
        }
        std::vector<Real> row{static_cast<Real>(step), step * dt,
                              f.steer, input.throttle, input.brake, input.handbrake,
                              s.body.pos.x, s.body.pos.y, s.body.pos.z,
                              roll_angle(s.body.q), pitch_angle(s.body.q), yaw,
                              f.vel.u, f.vel.v, f.vel.w, f.vel.p, f.vel.q, f.vel.r,
                              speed, std::atan2(f.vel.v, f.vel.u), s.body.vel.z, a_long, speed * f.vel.r,
                              net.x, net.y, yaw_moment};
        for (std::size_t i = 0; i < kWheels; ++i)
        {
            const wheel_state &w = s.wheels[i];
            row.insert(row.end(), {w.omega, steady_slip_ratio(p.wheel, w.omega, f.patch[i].vx),
                                   std::atan(steady_lateral_slip(p.wheel, w.omega, f.patch[i])), f.fz[i], f.tyre[i].fx,
                                   f.tyre[i].fy, tyre_grip(p.tyre(i), f.fz[i]), f.torque[i].drive, f.torque[i].brake,
                                   f.corner[i].deflection, f.corner[i].deflection_rate, f.corner[i].spring,
                                   f.corner[i].damper, f.corner[i].tyre_deflection, s.corners[i].z, f.corner[i].stop});
        }
        const stored_energy e = energy_of(s, p);
        const energy_work &w = books.work();
        row.insert(row.end(), {e.translation, e.rotation, e.gravity, e.springs, e.wheels, e.total(), w.drive, w.brake,
                               w.damper, w.tyre, books.residual(e)});
        return row;
    }
}
