#pragma once

#include "types/types.hpp"

namespace T1::ft
{

    // Everything below the F/T measurement plane: magnetic mount + brush.
    struct ToolGravityParams
    {
        double mass = 0.0;        // kg
        Vec3 com = Vec3::Zero();  // m, ft_sensor frame
        Wrench bias;              // sensor zero offset, ft_sensor frame
    };

    // Reading model: raw = external + gravity_wrench + bias  (all in ft_sensor frame).
    // gravity_in_ft: gravity vector [m/s^2] expressed in the ft_sensor frame.
    Wrench gravity_wrench(const ToolGravityParams& params, const Vec3& gravity_in_ft);

    // Returns the external wrench, ft_sensor frame.
    Wrench remove_gravity_and_bias(const Wrench& raw, const ToolGravityParams& params,
                                   const Vec3& gravity_in_ft);

}  // namespace T1::ft
