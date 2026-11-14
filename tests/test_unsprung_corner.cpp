#include <cmath>
#include "core/rigid_body.hpp"
#include "tests/check.hpp"
#include "vehicle/corner.hpp"

using namespace rallysim;

// The force law of a corner with a wheel mass, a tyre spring and a droop stop.
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

    // the bump stop's law: k = 500000, L = 0.03
    {
        const Real k = 500000, L = 0.03;
        check_near("bump stop force at 0.015", bump_stop_force(0.015, k, L), 1875, 1e-9);
        check_near("bump stop force at 0.03", bump_stop_force(0.03, k, L), 7500, 1e-9);
        check_near("bump stop force at 0.06", bump_stop_force(0.06, k, L), 22500, 1e-9);
        check_near("bump stop energy at 0.015", bump_stop_energy(0.015, k, L), 9.375, 1e-9);
        check_near("bump stop energy at 0.03", bump_stop_energy(0.03, k, L), 75, 1e-9);
        check_near("bump stop energy at 0.06", bump_stop_energy(0.06, k, L), 525, 1e-9);
        check_true("bump stop: nothing at 0 and before",
                   bump_stop_force(0, k, L) == 0 && bump_stop_energy(0, k, L) == 0 && bump_stop_force(-0.01, k, L) == 0 &&
                       bump_stop_energy(-0.01, k, L) == 0);
        const Real h = 1e-6;
        check_true("bump stop: smooth contact", (bump_stop_force(h, k, L) - bump_stop_force(0, k, L)) / h < 1e-4 * k);
        check_near("bump stop: stiffness below L", (bump_stop_force(L, k, L) - bump_stop_force(L - h, k, L)) / h, k, 1e-4 * k);
        check_near("bump stop: stiffness above L", (bump_stop_force(L + h, k, L) - bump_stop_force(L, k, L)) / h, k, 1e-4 * k);
        for (const Real e : {0.015, 0.03, 0.06})
        {
            const Real F = bump_stop_force(e, k, L);
            const Real dU = (bump_stop_energy(e + 1e-7, k, L) - bump_stop_energy(e - 1e-7, k, L)) / 2e-7;
            check_near("bump stop: the energy's slope is the force", dU, F, 1e-6 * F);
        }
    }

    // the bump stop in the corner, moved to 0.10 m past rest
    {
        corner_params b = cp;
        b.bump_travel = 0.10;
        const corner_load in = unsprung_corner_force(b, x0 + 0.10 + 0.03, 0.0, x0, 0.01, 0.0);
        check_near("bump stop in the corner: stop", in.stop, 7500, 1e-6);
        check_near("bump stop in the corner: strut = spring + stop", in.strut, 50000 * 0.1787 + 7500, 1e-6);
        check_true("bump stop in the corner: zero at its start", unsprung_corner_force(b, x0 + 0.10, 0.0, x0, 0.01, 0.0).stop == 0);
    }

    // the bump stop uses its own rate and length, not the droop stop's rate
    {
        corner_params b = cp;
        b.bump_travel = 0.10;
        b.bump_stop_rate = 300000;
        b.bump_stop_length = 0.02;
        check_near("own bump stop: in the ramp", unsprung_corner_force(b, x0 + 0.10 + 0.02, 0.0, x0, 0.01, 0.0).stop,
                   3000, 1e-6);
        check_near("own bump stop: past the ramp", unsprung_corner_force(b, x0 + 0.10 + 0.05, 0.0, x0, 0.01, 0.0).stop,
                   12000, 1e-6);
    }

    // the defaults never reach the bump stop below 0.25 m past rest
    {
        bool untouched = true;
        for (int i = 0; 0.001 * i <= x0 + 0.24; ++i)
            if (unsprung_corner_force(cp, 0.001 * i, 0.0, x0, 0.01, 0.0).stop > 0)
                untouched = false;
        check_true("defaults: no bump stop up to x0 + 0.24", untouched);
    }

    return finish("unsprung_corner");
}
