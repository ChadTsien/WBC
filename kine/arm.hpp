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

        void update(const ArmVec& q);

        Iso3 frame_pose(int frame_id) const;  // 在 base_link 下
        Iso3 tool_pose() const;
        Iso3 ft_pose() const;
        /**
         * 刷头在臂座（arm_base_link / arm_mount_joint）下的位姿
         *
         * 说明
         * - T_mount_tip = T_base_mount⁻¹ · T_base_tip
         */
        Iso3 tool_pose_in_mount() const;

        // Rows [linear; angular], axes of base_link, reference point at the frame origin.
        ArmJacobian frame_jacobian(int frame_id) const;
        ArmJacobian tool_jacobian() const;

        // sqrt(det(J J^T)) of the 5-row task Jacobian: linear rows plus the two angular rows
        // perpendicular to the brush axis.
        double manipulability() const;

        private:
        ArmModel& model_;
    };

}  // namespace T1
