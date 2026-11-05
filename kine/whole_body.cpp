#include "kine/whole_body.hpp"

namespace T1
{

    WholeBodyJacobian whole_body_jacobian(const ChassisModel& /*chassis*/,
                                          const Pose2d& /*base_pose*/, const Iso3& /*tool_in_base*/,
                                          const ArmJacobian& /*arm_jacobian*/)
    {
        // TODO(M1)
        return WholeBodyJacobian::Zero();
    }

}  // namespace T1
