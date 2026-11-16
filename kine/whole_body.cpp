#include "kine/whole_body.hpp"

#include <cmath>
#include "types/types.hpp"

namespace T1
{

    WholeBodyJacobian whole_body_jacobian(const ChassisModel& chassis,
                                          const Pose2d& chassis_pose, const Iso3& tool_in_base,
                                          const ArmJacobian& arm_jacobian)
    {
        const double c = std::cos(chassis_pose.yaw);
        const double s = std::sin(chassis_pose.yaw);
        Mat3 R = Mat3::Zero();
        R(0, 0) = c;
        R(0, 1) = -s;
        R(1, 0) = s;
        R(1, 1) = c;
        R(2, 2) = 1.0;

        WholeBodyJacobian j;
        j.leftCols<2>() = chassis.point_jacobian(tool_in_base.translation(), chassis_pose.yaw);
        j.block<3, 6>(0, 2) = R * arm_jacobian.topRows<3>();
        j.block<3, 6>(3, 2) = R * arm_jacobian.bottomRows<3>();
        return j;
    }

}  // namespace T1
