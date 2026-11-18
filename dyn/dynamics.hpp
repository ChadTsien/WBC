#pragma once

#include "model/arm_model.hpp"
#include "types/types.hpp"

namespace T1
{

    /**
     * 机械臂动力学项，给 MIT 模式做前馈
     *
     * 说明
     * - 写 ArmModel::data，与运动学共用这一份，不要并发
     * - gravity_torque 每拍可调；set_tool_payload 只在换刷时调
     */
    class Dynamics
    {
        public:
            explicit Dynamics(ArmModel& model);

            /**
             * 把 F/T 以下的工具负载并进最后一个臂连杆
             *
             * 参数
             * - mass：kg
             * - com_in_ft：质心在 ft_sensor 系
             *
             * 说明
             * - 与原始连杆惯量相加，不能在当前值上累加
             * - M3a 实现；现在是空操作
             */
            void set_tool_payload(double mass, const Vec3& com_in_ft);

            /**
             * 广义重力矩，顺序与 ArmVec 相同
             *
             * 参数
             * - q：当前关节
             * - gravity_in_base：重力加速度在 base_link，m/s²（IMU 已含坡度）
             *
             * 说明
             * - 直接写入 tau_ff；J3 为力（N），其余为力矩（N·m）
             * - q 非有限时返回 0
             */
            ArmVec gravity_torque(const ArmVec& q, const Vec3& gravity_in_base);

        private:
            ArmModel& model_;
    };

}  // namespace T1
