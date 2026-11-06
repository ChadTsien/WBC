#pragma once

#include "model/arm_model.hpp"
#include "types/types.hpp"

namespace T1
{

    using ArmJacobian = Eigen::Matrix<double, 6, kNumArmJoints>;

    /**
     * 机械臂正运动学与雅可比（相对 base_link）
     *
     * 说明
     * - 与 Dynamics 共用 ArmModel::data()
     * - 每个周期先 update(q)，取走位姿和雅可比，再调用 Dynamics
     */
    class ArmKinematics
    {
        public:
            explicit ArmKinematics(ArmModel& model);

            /**
            * 按 q 写入 ArmModel::data()：连杆位姿和关节雅可比
            *
            * 说明
            * - 每个周期只调一次，再取 pose / jacobian
            * - q 下标与 ArmModelConfig::arm_joints 相同
            */
            void update(const ArmVec& q);

            /**
            * 坐标系在 base_link 下的位姿
            *
            * 说明
            * - 读的是上一拍 update() 写进 data.oMf 的结果
            * - frame_id 用 ArmModel 解析好的 id，不要在循环里查字符串
            */
            Iso3 frame_pose(int frame_id) const;
            Iso3 tool_pose() const;
            Iso3 ft_pose() const;

            /**
            * 刷头在臂座（arm_base_link / arm_mount_joint）下的位姿
            *
            * 说明
            * - T_mount_tip = T_base_mount⁻¹ · T_base_tip
            */
            Iso3 tool_pose_in_mount() const;

            /**
            * 坐标系速度对 dq 的 6×6 雅可比（base_link 坐标轴）
            *
            * 说明
            * - 前 3 行线速度，后 3 行角速度
            * - 参考点在该坐标系原点；轴与 base_link 平行（LOCAL_WORLD_ALIGNED）
            */
            ArmJacobian frame_jacobian(int frame_id) const;
            ArmJacobian tool_jacobian() const;

            // sqrt(det(J J^T)) of the 5-row task Jacobian: linear rows plus the two angular rows
            // perpendicular to the brush axis.
            double manipulability() const;

        private:
            ArmModel& model_;
    };

}  // namespace T1
