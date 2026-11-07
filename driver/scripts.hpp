#pragma once
#include "core/wrench.hpp"
#include "driver/keyframe_script.hpp"
#include "vehicle/bicycle.hpp"
#include "vehicle/single_track.hpp"

namespace rallysim
{
    // The scripted inputs of each model: one keyframe_script per input type.
    using wrench_script = keyframe_script<wrench>;                    // force and torque on a rigid body
    using bicycle_script = keyframe_script<bicycle_input>;            // steer and acceleration
    using single_track_script = keyframe_script<single_track_input>;  // steer and drive force
}
