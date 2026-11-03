#pragma once

#include "model/robot_model.hpp"
#include "types/types.hpp"

namespace T1
{

    using ArmJacobian = Eigen::Matrix<double, 6, kNumArmJoints>;

    // Forward kinematics and Jacobians in base_link. Shares RobotModel::data with Dynamics:
    // call update() first in each cycle, then read; Dynamics calls overwrite cached results.
    class ArmKinematics
    {
        public:
        explicit ArmKinematics(RobotModel& model);

        void update(const ArmVec& q);

        Iso3 frame_pose(int frame_id) const;  // in base_link
        Iso3 tool_pose() const;
        Iso3 ft_pose() const;

        // Rows [linear; angular], axes of base_link, reference point at the frame origin.
        ArmJacobian frame_jacobian(int frame_id) const;
        ArmJacobian tool_jacobian() const;

        // sqrt(det(J J^T)) of the 5-row task Jacobian: linear rows plus the two angular rows
        // perpendicular to the brush axis.
        double manipulability() const;

        private:
        RobotModel& model_;
    };

}  // namespace T1
