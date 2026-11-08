#pragma once
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include "vehicle/tyre.hpp"

namespace rallysim
{
    // The single-tyre rig (book section 7.2): every curve swept over slip ratio, slip angle and load, with no car
    // and no time, so the force curves can be plotted and checked before a tyre goes on the car.
    //
    // Curves: linear (the default tyre), brush (peak_slide with its peak at 3 and no fall-off, which is the brush
    // model of section 17.7), peak_slide with its default shape, and tmeasy with its default tables. Sweeps:
    //   x         slip ratio -1 to 1, no slip angle, at each load
    //   y         slip angle -30 to 30 degrees, no slip ratio, at each load
    //   combined  slip ratio -1 to 1 at slip angles of 2, 4, 6 and 8 degrees, at the nominal load
    //
    // Output (its own format, not telemetry): a "# tyre_rig" line, the header
    //   curve,sweep,fz,slip_ratio,slip_angle_deg,fx,fy
    // then one row per point. fx and fy are in the wheel frame (N); slip_angle_deg is the angle whose tangent is
    // the lateral slip that tyre_forces takes.
    inline std::vector<std::pair<std::string, tyre_params>> tyre_rig_curves()
    {
        tyre_params linear;
        tyre_params brush;
        brush.curve = tyre_curve::peak_slide;
        brush.peak_slip = 3;
        brush.sliding_ratio = 1;
        tyre_params peak_slide;
        peak_slide.curve = tyre_curve::peak_slide;
        tyre_params tmeasy;
        tmeasy.curve = tyre_curve::tmeasy;
        return {{"linear", linear}, {"brush", brush}, {"peak_slide", peak_slide}, {"tmeasy", tmeasy}};
    }

    inline bool write_tyre_rig(const std::string &path)
    {
        std::ofstream out(path);
        if (!out)
            return false;
        out << "# tyre_rig\ncurve,sweep,fz,slip_ratio,slip_angle_deg,fx,fy\n";

        auto row = [&](const std::string &curve, const tyre_params &t, const char *sweep, Real fz, Real slip_ratio,
                       Real angle_deg)
        {
            const tyre_force f = tyre_forces(t, slip_ratio, std::tan(angle_deg * kPi / 180), fz);
            char line[160];
            std::snprintf(line, sizeof line, "%s,%s,%.0f,%.4f,%.2f,%.6g,%.6g\n", curve.c_str(), sweep, fz, slip_ratio,
                          angle_deg, f.fx, f.fy);
            out << line;
        };

        const Real loads[] = {1500, 3000, 4500, 6000};
        const Real combined_angles[] = {2, 4, 6, 8};
        for (const auto &[name, t] : tyre_rig_curves())
        {
            for (Real fz : loads)
                for (int i = -500; i <= 500; ++i) // slip ratio in steps of 0.002
                    row(name, t, "x", fz, i * 0.002, 0);
            for (Real fz : loads)
                for (int i = -300; i <= 300; ++i) // slip angle in steps of 0.1 degree
                    row(name, t, "y", fz, 0, i * 0.1);
            for (Real angle : combined_angles)
                for (int i = -500; i <= 500; ++i)
                    row(name, t, "combined", t.nominal_load, i * 0.002, angle);
        }
        out.close();
        return !out.fail();
    }
}
