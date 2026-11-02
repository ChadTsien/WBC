#pragma once

#include "signal/gravity.hpp"

namespace T1::ft {

// Removes the force m*a (and its moment about the sensor) produced by accelerating the tool.
// Tool rotational inertia is neglected (small round brush).
// com_accel_in_ft: linear acceleration of the tool CoM, ft_sensor frame, gravity excluded.
Wrench remove_inertia(const Wrench& wrench, const ToolGravityParams& params,
                      const Vec3& com_accel_in_ft);

}  // namespace T1::ft
