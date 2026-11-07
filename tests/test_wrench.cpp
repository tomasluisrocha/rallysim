#include <cmath>
#include <cstdio>
#include "core/rigid_body.hpp"
#include "core/wrench.hpp"
#include "tests/check.hpp"

using namespace rallysim;

static const Real pi = 3.14159265358979323846;

int main()
{
    const quat id;
    const quat z90 = from_axis_angle(vec3{0, 0, 1}, pi / 2);

    // an empty wrench is zero
    {
        wrench w;
        check_vec("default force", w.force, vec3{0, 0, 0});
        check_vec("default torque", w.torque, vec3{0, 0, 0});
    }

    // a force through the centre of mass adds force and no torque
    {
        wrench w;
        add_force_at(w, id, vec3{10, 0, 0}, vec3{0, 0, 0});
        check_vec("com: force", w.force, vec3{10, 0, 0});
        check_vec("com: no torque", w.torque, vec3{0, 0, 0});
    }

    // a force along the line through the centre of mass also adds no torque
    {
        wrench w;
        add_force_at(w, id, vec3{10, 0, 0}, vec3{3, 0, 0});
        check_vec("collinear: no torque", w.torque, vec3{0, 0, 0});
    }

    // sign checks, right-hand rule (x forward, y left, z up)
    {
        wrench w;
        add_force_at(w, id, vec3{10, 0, 0}, vec3{0, 1, 0}); // push forward on the left side
        check_vec("left side pushed forward: turns right (-z)", w.torque, vec3{0, 0, -10});
    }
    {
        wrench w;
        add_force_at(w, id, vec3{0, 10, 0}, vec3{2, 0, 0}); // push left at the front
        check_vec("front pushed left: turns left (+z)", w.torque, vec3{0, 0, 20});
    }
    {
        wrench w;
        add_force_at(w, id, vec3{0, 0, 10}, vec3{0, 1, 0}); // lift on the left side
        check_vec("left side lifted: rolls right (+x)", w.torque, vec3{10, 0, 0});
    }
    {
        wrench w;
        add_force_at(w, id, vec3{0, 0, 10}, vec3{2, 0, 0}); // lift at the front
        check_vec("front lifted: pitches nose up (-y)", w.torque, vec3{0, -20, 0});
    }

    // two equal and opposite forces (a couple): pure torque, no net force
    {
        wrench w;
        add_force_at(w, id, vec3{5, 0, 0}, vec3{0, 1, 0});
        add_force_at(w, id, vec3{-5, 0, 0}, vec3{0, -1, 0});
        check_vec("couple: no net force", w.force, vec3{0, 0, 0});
        check_vec("couple: torque", w.torque, vec3{0, 0, -10});
    }

    // contributions add up
    {
        wrench w;
        add_force_at(w, id, vec3{1, 2, 3}, vec3{0, 0, 0});
        add_force_at(w, id, vec3{4, 5, 6}, vec3{0, 0, 0});
        add_torque(w, vec3{0, 0, 7});
        add_torque(w, vec3{1, 0, 1});
        check_vec("force adds up", w.force, vec3{5, 7, 9});
        check_vec("torque adds up", w.torque, vec3{1, 0, 8});
    }

    // rotated body: the force is in the world frame, the point and the torque in the body frame
    {
        // body x axis points along world +y. A world +x force is then along body -y.
        wrench w;
        add_force_at(w, z90, vec3{10, 0, 0}, vec3{0, 1, 0});
        check_vec("rotated: world force stays world", w.force, vec3{10, 0, 0});
        check_vec("rotated: force along the line through the com: no torque", w.torque, vec3{0, 0, 0});
    }
    {
        // point on the body x axis, which is world +y; world +x force there turns the body right
        wrench w;
        add_force_at(w, z90, vec3{10, 0, 0}, vec3{1, 0, 0});
        check_vec("rotated: world +x force at body-front point turns right", w.torque, vec3{0, 0, -10});
    }

    // the torque is the same however the body is oriented, once the point is in the body frame:
    // the same physical push on the body's left side, seen from two orientations
    {
        wrench a, b;
        add_force_at(a, id, vec3{0, 0, 10}, vec3{0, 1, 0});                      // lift on left side
        add_force_at(b, z90, to_world(z90, vec3{0, 0, 10}), vec3{0, 1, 0});        // same force, body turned
        check_vec("same push, rotated body: same body-frame torque", a.torque, b.torque);
    }

    // end to end: an off-centre push on a free body both moves it and spins it.
    // The push is constant in the BODY frame (like a thruster fixed to the body), so its torque is
    // constant: tau = -(0.5 m * 8 N) = -4 N m about z, alpha = -4 / 4 = -1 rad/s^2.
    // (A push fixed in the WORLD frame would weaken as the body turns, since the lever arm shrinks.)
    {
        rigid_body_params p;
        p.mass = 2.0;
        p.inertia = {1.0, 1.0, 4.0};
        rigid_body_state s;
        const Real dt = 0.001;
        for (int i = 0; i < 1000; ++i)
        {
            wrench w;
            add_force_at(w, s.q, vec3{0, 0, p.mass * kGravity}, vec3{0, 0, 0});      // cancel gravity
            add_force_at(w, s.q, to_world(s.q, vec3{8, 0, 0}), vec3{0, 0.5, 0});       // body-frame push
            step(s, p, w, dt);
        }
        check_near("end to end: omega.z = tau / I * t", s.omega.z, -1.0, 1e-9);
        check_near("end to end: no roll or pitch", std::fabs(s.omega.x) + std::fabs(s.omega.y), 0.0, 1e-9);
        check_near("end to end: norm(q) = 1", norm(s.q), 1.0);
        check_near("end to end: it also moved forward (vel.x > 2)", s.vel.x > 2.0 ? 1.0 : 0.0, 1.0);
        check_near("end to end: speed <= 4 m/s (a = F/m = 4 m/s^2 for 1 s)", norm(s.vel) <= 4.0 + 1e-9 ? 1.0 : 0.0, 1.0);
    }

    return finish("wrench");
}