#pragma once
#include <algorithm>
#include <cmath>
#include "core/rigid_body.hpp"

namespace rallysim
{
    // A driver that steers the car to a heading, sliding or not, and countersteers to catch a slide. Not part of
    // any car model; the scripted scenarios use it where an open-loop script cannot work (the hairpin).
    //
    // Why it is needed: past the tyres' peak, a sliding car loses grip as it slides further, so a slide either
    // stays small or runs away into a spin. A fixed script has to land inside that gap by luck: for the hairpin, a
    // handbrake pull of 0.73 turned the car 83 degrees, 0.74 turned it 171 and 0.75 spun it, and any change to the
    // physics would have moved that edge. A driver reacts; with this controller the hairpin turns 180 degrees for a
    // pull from 0.5 to 1.0, a steer from 0.3 to 0.5 rad and on either peaked tyre.
    //
    // How it steers:
    //   1. the yaw rate it wants, proportional to the heading still to go, capped:
    //        r_wanted = clamp(heading_gain * (target - heading), +-max_yaw_rate)
    //   2. the steer: the front wheels aimed where the front axle is travelling, plus a correction towards that yaw
    //      rate:
    //        steer = atan2(v + lf r, u) + yaw_rate_gain * (r_wanted - r)
    // The first term is countersteer. A sliding car moves at an angle to where it points; a front tyre only gives a
    // useful force when it is aimed close to its own direction of travel, so the wheels are pointed along the slide
    // and the correction then steers into or out of it. (Steering as if the car gripped, L r / u, turned it the wrong
    // way: the front tyres were aimed far off their travel and saturated.) The forward speed in the angle is taken
    // as at least 1 m/s, so a car stopped or going backwards still gets a finite steer.
    struct heading_hold_params
    {
        Real heading_gain = 2.5;  // 1/s: yaw rate asked for per radian of heading still to go
        Real max_yaw_rate = 1.2;  // rad/s: the most it asks for (larger turns the hairpin into a bigger slide)
        Real yaw_rate_gain = 0.35; // rad of steer per rad/s of yaw rate error
    };

    // The heading error wrapped into (-pi, pi], so the driver turns the short way round.
    inline Real heading_error(Real target, Real heading)
    {
        Real e = std::fmod(target - heading, 2 * kPi);
        if (e > kPi)
            e -= 2 * kPi;
        if (e <= -kPi)
            e += 2 * kPi;
        return e;
    }

    // The steer (rad, +left) that turns a car moving at `v` (body frame) towards `target` (rad, world heading),
    // with the front axle `lf` (m) ahead of the CG, limited to +-max_steer.
    inline Real heading_hold_steer(const body_velocity &v, Real heading, Real target, Real lf, Real max_steer,
                                   const heading_hold_params &c = {})
    {
        const Real r_wanted = std::clamp(c.heading_gain * heading_error(target, heading), -c.max_yaw_rate, c.max_yaw_rate);
        const Real front_travel = std::atan2(v.v + lf * v.r, std::max(v.u, static_cast<Real>(1)));
        return std::clamp(front_travel + c.yaw_rate_gain * (r_wanted - v.r), -max_steer, max_steer);
    }
}
