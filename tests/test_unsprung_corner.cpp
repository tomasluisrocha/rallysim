#include <cmath>
#include "core/rigid_body.hpp"
#include "tests/check.hpp"
#include "vehicle/corner.hpp"

using namespace rallysim;

// Task 6.2a: the force law of a corner with a wheel mass, a tyre spring and a droop stop.
// Default corner_params; every expected value is the formula worked by hand.

int main()
{
    const corner_params cp;
    const Real x0 = 0.0487;

    {
        const corner_load l = unsprung_corner_force(cp, 0.05, 0.1, x0, 0.01, 0.0);
        check_near("compressed: spring", l.spring, 2500, 1e-9);
        check_near("compressed: damper", l.damper, 380, 1e-9);
        check_true("compressed: stop is zero", l.stop == 0);
        check_near("compressed: strut", l.strut, 2880, 1e-9);
        check_near("compressed: tyre load", l.fz, 2500, 1e-9);
        check_true("compressed: inputs stored as given",
                   l.deflection == 0.05 && l.deflection_rate == 0.1 && l.tyre_deflection == 0.01 &&
                       l.tyre_deflection_rate == 0.0);
    }

    {
        const corner_load l = unsprung_corner_force(cp, 0.001, -1.0, x0, 0.01, 0.0);
        check_near("the strut may pull", l.strut, -3750, 1e-9);
        const corner_load m = corner_force(cp, 0.001, -1.0);
        check_true("the massless corner does not pull", m.fz == 0);
    }

    {
        const corner_load l = unsprung_corner_force(cp, -0.02, 0.0, x0, 0.01, 0.0);
        check_true("past the free length: no spring, no stop, no strut", l.spring == 0 && l.stop == 0 && l.strut == 0);
    }

    {
        const corner_load at = unsprung_corner_force(cp, x0 - 0.10, 0.0, x0, 0.01, 0.0);
        check_true("at the stop's start: no stop force", at.stop == 0);
        const corner_load in = unsprung_corner_force(cp, x0 - 0.11, 0.0, x0, 0.01, 0.0);
        check_near("past the stop's start: stop force", in.stop, -5000, 1e-9);
        check_near("past the stop's start: strut", in.strut, -5000, 1e-9);
        bool never_positive = true;
        for (int i = 0; i <= 400; ++i)
        {
            const Real x = -0.2 + 0.001 * i;
            if (unsprung_corner_force(cp, x, 0.0, x0, 0.01, 0.0).stop > 0)
                never_positive = false;
        }
        check_true("the stop is never positive", never_positive);
    }

    {
        const Real x = x0 - 0.10 - 40 * kGravity / 500000;
        const corner_load l = unsprung_corner_force(cp, x, 0.0, x0, 0.01, 0.0);
        check_near("hanging wheel: the stop carries its weight", l.strut, -40 * kGravity, 1e-9);
    }

    {
        corner_params d = cp;
        d.tyre_damping = 1000;
        const corner_load off = unsprung_corner_force(d, 0.0, 0.0, x0, -0.001, 1.0);
        check_true("tyre off the ground: no load", off.fz == 0);
        const corner_load pull = unsprung_corner_force(d, 0.0, 0.0, x0, 0.001, -1.0);
        check_true("tyre damping never pulls", pull.fz == 0);
        const corner_load add = unsprung_corner_force(d, 0.0, 0.0, x0, 0.01, 0.5);
        check_near("tyre damping adds", add.fz, 3000, 1e-9);
    }

    {
        const corner_load l = corner_force(cp, 0.05, 0.1);
        check_near("corner_force: spring", l.spring, 2500, 1e-9);
        check_near("corner_force: damper", l.damper, 380, 1e-9);
        check_near("corner_force: fz", l.fz, 2880, 1e-9);
        check_true("corner_force: new fields are zero",
                   l.stop == 0 && l.strut == 0 && l.tyre_deflection == 0 && l.tyre_deflection_rate == 0);
    }

    // rebound damping three times the bump damping
    {
        corner_params rp = cp;
        rp.rebound_ratio = 3;
        const corner_load bump = unsprung_corner_force(rp, 0.05, 0.2, x0, 0.01, 0.0);
        check_near("rebound ratio 3: bump damper", bump.damper, 760, 1e-9);
        const corner_load reb = unsprung_corner_force(rp, 0.05, -0.2, x0, 0.01, 0.0);
        check_near("rebound ratio 3: rebound damper", reb.damper, -2280, 1e-9);
        check_near("rebound ratio 3: rebound strut", reb.strut, 220, 1e-9);
        check_true("rebound ratio 3: rate zero gives no damper", unsprung_corner_force(rp, 0.05, 0.0, x0, 0.01, 0.0).damper == 0);
        const corner_load up = unsprung_corner_force(rp, 0.05, 1e-9, x0, 0.01, 0.0);
        const corner_load dn = unsprung_corner_force(rp, 0.05, -1e-9, x0, 0.01, 0.0);
        check_true("rebound ratio 3: continuous at zero", std::fabs(up.damper) < 2e-5 && std::fabs(dn.damper) < 2e-5);
        bool takes_energy = true;
        for (int i = 0; i <= 400; ++i)
        {
            const Real v = -2 + 0.01 * i;
            if (unsprung_corner_force(rp, 0.05, v, x0, 0.01, 0.0).damper * v < 0)
                takes_energy = false;
        }
        check_true("rebound ratio 3: the damper always takes energy out", takes_energy);
        check_near("default ratio: rebound damper unchanged", unsprung_corner_force(cp, 0.05, -0.2, x0, 0.01, 0.0).damper, -760, 1e-9);
        check_near("springs corner ignores the ratio", corner_force(rp, 0.05, -0.1).fz, 2120, 1e-9);
    }

    return finish("unsprung_corner");
}
