#pragma once

#include "kine/arm.hpp"
#include "model/chassis_model.hpp"
#include "types/types.hpp"

namespace T1
{

    using WholeBodyJacobian = Eigen::Matrix<double, 6, kNumWbcDof>;

    // Tool twist [linear; angular] in world axes as a function of [v, omega, dq1..dq6].
    // arm_jacobian: ArmKinematics::tool_jacobian() (base_link axes).
    WholeBodyJacobian whole_body_jacobian(const ChassisModel& chassis, const Pose2d& base_pose,
                                          const Iso3& tool_in_base,
                                          const ArmJacobian& arm_jacobian);

}  // namespace T1
