#pragma once

#include "control/types.hpp"

namespace T1::wbc
{

    // Interpolated reference at time t [s from trajectory start]. Positions / forces linear,
    // orientations slerp, discrete fields (modes, phases) from the earlier sample.
    // Before the first sample: first sample. After the last: last sample with zero velocities.
    // traj must be non-empty and sorted by t. No allocation.
    Reference sample_reference(const Trajectory& traj, double t);

}  // namespace T1::wbc
