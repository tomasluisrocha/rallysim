#include <cmath>
#include <cstdio>
#include "tests/check.hpp"
#include "vehicle/tyre.hpp"

using namespace rallysim;

// The tyre on its own (for the rungs built so far): zero force at zero slip, force
// that opposes slip, force bounded by mu Fz, and left-right and drive-brake symmetry.
int main()
{
    const tyre_params t; // Cx 90 kN, C_alpha 30 kN/rad, mu 1, k 0.1, Fz0 3000 N
    const Real fz = 3000;

    // zero slip, zero force, at any load
    for (Real load : {0.0, 1000.0, 3000.0, 8000.0})
    {
        const tyre_force f = tyre_forces(t, 0.0, 0.0, load);
        check_true("zero slip gives exactly zero force", f.fx == 0.0 && f.fy == 0.0);
    }

    // linear range: each force is stiffness times its own slip
    {
        const tyre_force f = tyre_forces(t, 0.01, 0.02, fz);
        check_near("linear: Fx = Cx * slip ratio", f.fx, 90000 * 0.01, 1e-9);
        check_near("linear: Fy = C_alpha * lateral slip", f.fy, 30000 * 0.02, 1e-9);
    }

    // force opposes slip: positive slip (wheel faster than the road, or pointing left of its path) gives a
    // positive force, which slows the wheel and pushes the car left
    {
        check_true("driving slip gives a forward force", tyre_forces(t, 0.05, 0.0, fz).fx > 0);
        check_true("braking slip gives a backward force", tyre_forces(t, -0.05, 0.0, fz).fx < 0);
        check_true("positive slip angle gives a force to the left", tyre_forces(t, 0.0, 0.05, fz).fy > 0);
        check_true("negative slip angle gives a force to the right", tyre_forces(t, 0.0, -0.05, fz).fy < 0);
    }

    // symmetry: mirroring the slip mirrors the force exactly
    for (Real sx : {0.003, 0.05, 0.4})
        for (Real sy : {0.002, 0.04, 0.3})
        {
            const tyre_force a = tyre_forces(t, sx, sy, fz);
            const tyre_force left_right = tyre_forces(t, sx, -sy, fz);
            const tyre_force drive_brake = tyre_forces(t, -sx, sy, fz);
            check_true("symmetry: lateral slip mirrored", left_right.fx == a.fx && left_right.fy == -a.fy);
            check_true("symmetry: slip ratio mirrored", drive_brake.fx == -a.fx && drive_brake.fy == a.fy);
        }

    // bounded: over a grid of slips and loads the force never exceeds mu(Fz) Fz
    {
        bool bounded = true;
        for (Real load : {500.0, 3000.0, 6000.0, 12000.0})
            for (Real sx = -1.0; sx <= 1.0; sx += 0.05)
                for (Real sy = -1.0; sy <= 1.0; sy += 0.05)
                {
                    const tyre_force f = tyre_forces(t, sx, sy, load);
                    bounded = bounded && std::hypot(f.fx, f.fy) <= tyre_grip(t, load) * (1 + 1e-12);
                }
        check_true("bounded: |F| <= mu(Fz) Fz everywhere on the grid", bounded);
    }

    // saturated: the size is exactly the grip and the direction is the linear force's direction
    {
        const tyre_force f = tyre_forces(t, 0.5, 0.5, fz);
        check_near("saturated: |F| = grip", std::hypot(f.fx, f.fy), tyre_grip(t, fz), 1e-9);
        check_near("saturated: direction kept (Fy / Fx = C_alpha / Cx)", f.fy / f.fx, 30000.0 / 90000.0, 1e-12);

        // pure slip saturates on its own axis
        check_near("pure longitudinal slip saturates at the grip", tyre_forces(t, 1.0, 0.0, fz).fx, tyre_grip(t, fz), 1e-9);
        check_near("pure lateral slip saturates at the grip", tyre_forces(t, 0.0, -1.0, fz).fy, -tyre_grip(t, fz), 1e-9);

        // continuous at the knee: the slip where Cx sx = grip gives exactly the grip
        const Real knee = tyre_grip(t, fz) / 90000;
        check_near("continuous at the friction limit", tyre_forces(t, knee, 0.0, fz).fx, tyre_grip(t, fz), 1e-9);
    }

    // friction circle: braking uses up grip that cornering cannot have
    {
        const Real side_alone = tyre_forces(t, 0.0, 0.3, fz).fy;
        const tyre_force both = tyre_forces(t, -0.3, 0.3, fz);
        check_true("combined: braking lowers the side force at the same slip angle", std::fabs(both.fy) < std::fabs(side_alone));
    }

    // load: no load, no force; and load sensitivity
    {
        const tyre_force air = tyre_forces(t, 0.2, 0.2, 0.0);
        check_true("unloaded tyre (in the air) gives no force", air.fx == 0.0 && air.fy == 0.0);
        const tyre_force pulled = tyre_forces(t, 0.2, 0.2, -500.0);
        check_true("negative load gives no force", pulled.fx == 0.0 && pulled.fy == 0.0);

        check_near("mu at the nominal load", peak_friction(t, 3000.0), 1.0, 1e-12);
        check_near("mu at twice the nominal load", peak_friction(t, 6000.0), 0.9, 1e-12);
        check_near("grip at 6000 N = 0.9 * 6000", tyre_grip(t, 6000.0), 5400.0, 1e-9);
        check_true("more load, more grip, but less than proportionally",
                   tyre_grip(t, 6000.0) > tyre_grip(t, 3000.0) && tyre_grip(t, 6000.0) < 2 * tyre_grip(t, 3000.0));

        // the grip never falls as the load rises: mu is floored at mu0 (1 + k) / 2 = 0.55, its value at the
        // load where mu(fz) fz peaks, fz0 (1 + k) / (2 k) = 16.5 kN
        bool rising = true;
        for (int i = 0; i < 500; ++i)
            rising = rising && tyre_grip(t, 100.0 * (i + 1)) >= tyre_grip(t, 100.0 * i);
        check_true("grip never falls from 0 to 50 kN", rising);

        const Real knee = 16500.0, at_knee = 0.55 * knee; // 9075 N
        check_near("grip at the knee = 0.55 * 16500", tyre_grip(t, knee), at_knee, 1e-9);
        check_near("grip continuous just below the knee", tyre_grip(t, knee - 1e-10), at_knee, 1e-9);
        check_near("grip continuous just above the knee", tyre_grip(t, knee + 1e-10), at_knee, 1e-9);
        check_near("mu at 40 kN is the floor", peak_friction(t, 40000.0), 0.55, 1e-12);
        check_near("grip at 40 kN = 0.55 * 40000", tyre_grip(t, 40000.0), 22000.0, 1e-9);

        tyre_params slippery = t;
        slippery.friction_coefficient = 0.5;
        check_near("half the friction coefficient, half the grip", tyre_grip(slippery, 3000.0), 1500.0, 1e-9);
    }

    return finish("tyre");
}
