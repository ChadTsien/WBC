#pragma once

#include <memory>
#include <pinocchio/multibody/fwd.hpp>
#include <string>
#include <vector>

#include "types/types.hpp"

namespace T1
{
    /**
     * 机械臂建模配置
     *
     * 成员
     * - urdf_xml：展开后的 URDF；来自测试文件，或 robot_state_publisher 的
     *   robot_description 参数
     * - arm_joints：保留的 6 个关节，顺序即 ArmVec 下标
     * - tool_frame / ft_frame：刷头、力传感器坐标系名
     * - mount_frame：臂座，arm_mount_joint 的子连杆，默认 arm_base_link
     */
    struct ArmModelConfig
    {
        std::string urdf_xml;
        std::vector<std::string> arm_joints{"arm_joint1", "arm_joint2", "arm_joint3",
                                            "arm_joint4", "arm_joint5", "arm_joint6"};
        std::string tool_frame = "brush_tip";
        std::string ft_frame = "ft_sensor";
        std::string mount_frame = "arm_base_link";
    };

    /**
     * Pinocchio 机械臂模型（根在 base_link，能动的只有 6 个臂关节）
     *
     * 说明
     * - 输入是展开后的 URDF 文本，不是 xacro
     * - 轮子、相机偏航等非臂关节锁死，nq == 6
     * - chassis_joint / arm_mount_joint 为 fixed，安装位焊在树上
     * - 独占一份 pinocchio::Data，非线程安全
     */
    class ArmModel
    {
        public:
            /**
            * 从 URDF 建树并锁非臂关节
            *
            * 说明
            * - URDF 读失败或关节 / 坐标系名不对时抛 runtime_error
            */
            explicit ArmModel(const ArmModelConfig& config);
            ~ArmModel();
            ArmModel(const ArmModel&) = delete;
            ArmModel& operator=(const ArmModel&) = delete;

            const pinocchio::Model& model() const;
            pinocchio::Data& data();

            /** 工具头坐标系 */
            int tool_frame_id() const;
            /** 力传感器坐标系 */
            int ft_frame_id() const;
            /** 臂基座 arm_base_link */
            int mount_frame_id() const;
            
            /**
            * 按名字解析坐标系
            *
            * 说明
            * - 不存在则抛 runtime_error
            * - 启动时解析一次，不要在 500 Hz 循环里按字符串查
            */
            int frame_id(const std::string& name) const;

            /**
            * 关节限位，顺序与 arm_joints 相同
            *
            * 说明
            * - J3 单位 m / m/s，其余 rad / rad/s
            */
            ArmVec lower_limits() const;
            ArmVec upper_limits() const;
            ArmVec velocity_limits() const;

        private:
            struct Impl;
            std::unique_ptr<Impl> impl_;
    };

}  // namespace T1
