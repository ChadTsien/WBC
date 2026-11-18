#pragma once

#include "types/types.hpp"

namespace T1::est
{

    /**
     * IMU 姿态 → 重力在 base_link 下的加速度
     *
     * 说明
     * - 给重力矩和力传感器补偿用；斜坡上方向会偏
     * - q：IMU 系在重力对齐系里的朝向
     * - base_R_imu：IMU 相对 base_link 的安装旋转
     * - yaw 对结果没有影响
     */
    Vec3 gravity_in_base(const Quat& imu_orientation, const Mat3& base_R_imu, double g = 9.81);

}  // namespace T1::est
