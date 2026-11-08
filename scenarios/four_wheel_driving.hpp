#pragma once
#include "scenarios/run_four_wheel.hpp"

namespace rallysim
{
    // Straight-line braking: 25 m/s, rolling for a second, then the brake pedal fully down until the end
    // (5 s). Default car: brake bias 70% front, strong enough to lock all four wheels. Locked, the peak_slide
    // tyre gives sliding_ratio of its grip, so the stop takes about 3.2 s and 40 m (2.6 s and 32 m on the
    // linear tyre); the run is long enough for either.
    inline four_wheel_run four_wheel_brake()
    {
        four_wheel_run r;
        r.initial = four_wheel_initial(r.params, 25.0);
        r.steps = 5000;
        r.script.keys = {{0.0, four_wheel_input{}},
                         {1.0, four_wheel_input{}},
                         {1.05, four_wheel_input{0.0, 0.0, 1.0, 0.0}},
                         {5.0, four_wheel_input{0.0, 0.0, 1.0, 0.0}}};
        return r;
    }

    // Launch: four-wheel drive with 30% of the torque on the front (about the front's share of the load while
    // accelerating hard), rolling at 2 m/s, full throttle from 0.5 s for 3 s. The wheels spin up, the car
    // accelerates at about 1 g, then the engine runs out of power and the acceleration falls as P / (m v).
    inline four_wheel_run four_wheel_launch()
    {
        four_wheel_run r;
        r.params.drive_brake.drive_split_front = 0.3;
        r.initial = four_wheel_initial(r.params, 2.0);
        r.steps = 3500;
        r.script.keys = {{0.0, four_wheel_input{}},
                         {0.5, four_wheel_input{}},
                         {0.55, four_wheel_input{0.0, 1.0, 0.0, 0.0}},
                         {3.5, four_wheel_input{0.0, 1.0, 0.0, 0.0}}};
        return r;
    }

    // Handbrake flick: 15 m/s held by the speed controller, steer into a gentle left turn at 1 s, then a short,
    // partial pull of the handbrake (0.36 for 0.3 s from 2.5 s) and the wheel straightened after it. The inner
    // rear locks, the rear steps out (about 7 degrees of sideslip) and the car recovers. A stronger or longer
    // pull also locks the outer rear (braking pitches the body forward and unloads the rear), and then the rear
    // axle has no side grip left: with the speed controller back on the throttle and no countersteer from the
    // script, this rear-drive car spins right round.
    //
    // On the peak_slide tyre (phase 5) the edge is sharp: a sliding rear loses grip as it slides more, so a slide
    // either stays small or runs away. 0.35 for 0.3 s gives 5 degrees, 0.39 gives 12, 0.40 spins; 0.36 for 0.4 s
    // spins too. On the linear tyre, which keeps its grip while sliding, 0.55 still recovered.
    inline four_wheel_run four_wheel_handbrake()
    {
        four_wheel_run r;
        r.initial = four_wheel_initial(r.params, 15.0);
        r.hold_speed = 15.0;
        r.steps = 6000;
        const Real steer = 0.05, pull = 0.36;
        r.script.keys = {{0.0, four_wheel_input{}},
                         {1.0, four_wheel_input{}},
                         {1.3, four_wheel_input{steer, 0.0, 0.0, 0.0}},
                         {2.5, four_wheel_input{steer, 0.0, 0.0, 0.0}},
                         {2.55, four_wheel_input{steer, 0.0, 0.0, pull}},
                         {2.85, four_wheel_input{steer, 0.0, 0.0, pull}},
                         {2.9, four_wheel_input{steer, 0.0, 0.0, 0.0}},
                         {3.2, four_wheel_input{0.0, 0.0, 0.0, 0.0}},
                         {6.0, four_wheel_input{0.0, 0.0, 0.0, 0.0}}};
        return r;
    }
}
