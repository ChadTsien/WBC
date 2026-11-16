#include "control/task_base.hpp"

namespace T1::wbc
{

    void ChassisTask::configure(const ChassisTaskConfig& config)
    {
        config_ = config;
    }

    void ChassisTask::add_to(QpProblem& /*qp*/, const Reference& /*ref*/,
                            const Pose2d& /*chassis_pose*/)
    {
        // TODO(M1)
    }

}  // namespace T1::wbc
