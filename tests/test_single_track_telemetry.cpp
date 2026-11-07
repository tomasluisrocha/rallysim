#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include "vehicle/single_track_telemetry.hpp"
#include "tests/check.hpp"
#include "tests/telemetry_helpers.hpp"
#include "tests/single_track_helpers.hpp"

using namespace rallysim;

int main()
{
    const single_track_params p; // 1200 kg, lf = 1.3, lr = 1.2, Cf = 60000, Cr = 70000, mu = 1, max_steer = 0.6
    const rigid_body_state s = car(1.5, -2.5, 0.75, 10.0, -0.3, 0.4);
    const single_track_input in{0.05, 800.0};

    const std::vector<std::string> names = single_track_columns();
    const std::vector<std::string> units = single_track_units();
    const std::vector<Real> row = single_track_row(5, 0.001, s, p, in);

    // sizes
    check_true("24 column names", names.size() == 24);
    check_true("24 units", units.size() == 24);
    check_true("24 values in a row", row.size() == 24);

    // names are unique
    for (std::size_t i = 0; i < names.size(); ++i)
        for (std::size_t j = i + 1; j < names.size(); ++j)
            check_true("column names are unique", names[i] != names[j]);

    // each value lands under the right name (every field has a different value, so a swap shows)
    check_column(names, row, "step", 5);
    check_column(names, row, "time", 5 * 0.001);
    check_column(names, row, "pos.x", 1.5, 1e-12);
    check_column(names, row, "pos.y", -2.5, 1e-12);
    check_column(names, row, "yaw", 0.75, 1e-12);
    check_column(names, row, "steer", 0.05);
    check_column(names, row, "drive_force", 800.0);
    check_column(names, row, "u", 10.0, 1e-12);
    check_column(names, row, "v", -0.3, 1e-12);
    check_column(names, row, "yaw_rate", 0.4, 1e-12);

    // derived values against closed forms written out here, not by calling the model's helpers
    const Real speed = std::hypot(10.0, -0.3);
    check_column(names, row, "speed", speed, 1e-12);
    check_column(names, row, "beta", std::atan2(-0.3, 10.0), 1e-12);
    check_column(names, row, "a_lat", speed * 0.4, 1e-12);

    const Real alpha_f = 0.05 - std::atan((-0.3 + 1.3 * 0.4) / 10.0);
    const Real alpha_r = -std::atan((-0.3 - 1.2 * 0.4) / 10.0);
    check_column(names, row, "axle.front.slipAngle", alpha_f, 1e-12);
    check_column(names, row, "axle.rear.slipAngle", alpha_r, 1e-12);
    check_column(names, row, "axle.front.Fy", 60000.0 * alpha_f, 1e-6); // below the friction limit
    check_column(names, row, "axle.rear.Fy", 70000.0 * alpha_r, 1e-6);

    // wheel loads: the axle loads with the longitudinal transfer, then the lateral transfer between the sides.
    // Here a_x = 800 / 1200 and a_y = u * r = 10 * 0.4 = 4, written out with the numbers of the car.
    {
        const Real weight = 1200.0 * kGravity;
        const Real a_x = 800.0 / 1200.0, a_y = 10.0 * 0.4;
        const Real front = weight * 1.2 / 2.5 - 1200.0 * a_x * 0.5 / 2.5;
        const Real rear = weight * 1.3 / 2.5 + 1200.0 * a_x * 0.5 / 2.5;
        const Real half_total = 0.5 * (1200.0 * a_y * 0.5 / 1.5); // the front axle's share of the lateral transfer
        check_column(names, row, "wheel.FL.Fz", front / 2 - half_total, 1e-6);
        check_column(names, row, "wheel.FR.Fz", front / 2 + half_total, 1e-6);
        check_column(names, row, "wheel.RL.Fz", rear / 2 - half_total, 1e-6);
        check_column(names, row, "wheel.RR.Fz", rear / 2 + half_total, 1e-6);

        Real total = 0;
        for (const char *name : {"wheel.FL.Fz", "wheel.FR.Fz", "wheel.RL.Fz", "wheel.RR.Fz"})
            total += row[static_cast<std::size_t>(index_of(names, name))];
        check_near("wheel loads in a row add up to the weight", total, weight, 1e-6);
    }

    // grip and rear Fx: mu(fz) fz summed over each axle's wheels, with mu(fz) = 1 - 0.1 (fz - 3000) / 3000,
    // written out here; the drive force of 800 N is far below the rear grip, so it passes unchanged
    {
        const auto wheel = [&](const char *name) { return row[static_cast<std::size_t>(index_of(names, name))]; };
        const auto grip = [](Real fz) { return (1.0 - 0.1 * (fz - 3000.0) / 3000.0) * fz; };
        check_column(names, row, "axle.front.grip", grip(wheel("wheel.FL.Fz")) + grip(wheel("wheel.FR.Fz")), 1e-6);
        check_column(names, row, "axle.rear.grip", grip(wheel("wheel.RL.Fz")) + grip(wheel("wheel.RR.Fz")), 1e-6);
        check_column(names, row, "axle.rear.Fx", 800.0);
    }

    // the logged steer is the limited value: 3 rad is logged as max_steer, the slip follows it, and with
    // that much slip the front axle is at its grip
    {
        const std::vector<Real> r = single_track_row(0, 0.001, s, p, single_track_input{3.0, 0.0});
        check_column(names, r, "steer", 0.6);
        check_column(names, r, "axle.front.slipAngle", 0.6 - std::atan((-0.3 + 1.3 * 0.4) / 10.0), 1e-12);
        check_column(names, r, "axle.front.Fy", r[static_cast<std::size_t>(index_of(names, "axle.front.grip"))], 1e-9);
    }

    // the logged forces use the loads with transfer, like the step: hard braking moves
    // m a_x h / L = 1200 * (-5000 / 1200) * 0.5 / 2.5 = -1000 N, so +1000 N to the front axle, 500 N per wheel.
    // With no lateral transfer here (yaw rate 0) each front wheel carries W lr / (2 L) + 500 N.
    {
        const std::vector<Real> r = single_track_row(0, 0.001, car(0, 0, 0, 10.0, -0.3, 0.0), p, single_track_input{3.0, -5000.0});
        const Real fz = 1200.0 * kGravity * 1.2 / 2.5 / 2 + 500.0;
        const Real wheel_grip = (1.0 - 0.1 * (fz - 3000.0) / 3000.0) * fz;
        check_column(names, r, "wheel.FL.Fz", fz, 1e-9);
        check_column(names, r, "axle.front.Fy", 2 * wheel_grip, 1e-6);
        check_column(names, r, "drive_force", -5000.0);
        check_column(names, r, "axle.rear.Fx", -5000.0); // below the rear grip, so passed unchanged
    }

    // below the minimum forward speed, u is logged as it is; only the slip angles use the floor
    {
        const std::vector<Real> r = single_track_row(0, 0.001, car(0, 0, 0, 0.2, 0.1, 0.0), p, single_track_input{});
        check_column(names, r, "u", 0.2, 1e-12);
        check_column(names, r, "axle.front.slipAngle", -std::atan(0.1 / kMinForwardSpeed), 1e-12);
    }

    // sign: a right turn gives a negative yaw rate and a negative a_lat
    {
        const std::vector<Real> r = single_track_row(0, 0.001, car(0, 0, 0, 10.0, 0.0, -0.4), p, single_track_input{-0.05, 0.0});
        check_true("right turn: yaw_rate < 0", r[static_cast<std::size_t>(index_of(names, "yaw_rate"))] < 0.0);
        check_true("right turn: a_lat < 0", r[static_cast<std::size_t>(index_of(names, "a_lat"))] < 0.0);
    }

    // units match the columns they describe
    auto check_unit = [&](const char *name, const char *unit)
    {
        const int i = index_of(names, name);
        check_true(name, i >= 0 && units[static_cast<std::size_t>(i)] == unit);
    };
    check_unit("step", "count");
    check_unit("time", "s");
    check_unit("pos.x", "m");
    check_unit("yaw", "rad");
    check_unit("speed", "m/s");
    check_unit("steer", "rad");
    check_unit("drive_force", "N");
    check_unit("u", "m/s");
    check_unit("v", "m/s");
    check_unit("yaw_rate", "rad/s");
    check_unit("beta", "rad");
    check_unit("a_lat", "m/s^2");
    check_unit("axle.front.slipAngle", "rad");
    check_unit("axle.rear.slipAngle", "rad");
    check_unit("axle.front.Fy", "N");
    check_unit("axle.rear.Fy", "N");
    check_unit("wheel.FL.Fz", "N");
    check_unit("wheel.FR.Fz", "N");
    check_unit("wheel.RL.Fz", "N");
    check_unit("wheel.RR.Fz", "N");
    check_unit("axle.front.grip", "N");
    check_unit("axle.rear.grip", "N");
    check_unit("axle.rear.Fx", "N");

    // the first two columns are step and time, as the schema says
    check_true("first column is step", names[0] == "step");
    check_true("second column is time", names[1] == "time");

    // tools/topdown.py reads a file as a vehicle if it has these columns
    for (const char *needed : {"pos.x", "pos.y", "yaw", "steer", "speed", "beta"})
        check_true(needed, index_of(names, needed) >= 0);

    return finish("single_track_telemetry");
}