#pragma once

#include "kine/arm.hpp"
#include "model/chassis_model.hpp"
#include "types/types.hpp"

namespace T1
{

    using WholeBodyJacobian = Eigen::Matrix<double, 6, kNumWbcDof>;

    /**
     * 工具空间速度（世界系坐标轴）对 WbcVec 的 6×8 雅可比
     *
     * 说明
     * - 列：[v, ω, dq1..dq6]
     * - 前 2 列：chassis.point_jacobian(工具在 base 下的位置, yaw)
     * - 后 6 列：blkdiag(R, R) · J_arm，R = Rz(yaw)，把 base 轴转到世界
     * - arm_jacobian 用 ArmKinematics::tool_jacobian()，须已 update(q)
     */
    WholeBodyJacobian whole_body_jacobian(const ChassisModel& chassis, const Pose2d& chassis_pose,
                                          const Iso3& tool_in_base,
                                          const ArmJacobian& arm_jacobian);

}  // namespace T1
