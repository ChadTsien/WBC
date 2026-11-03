#include "control/task_base.hpp"

namespace T1::wbc
{

    void BaseTask::configure(const BaseTaskConfig& config)
    {
        config_ = config;
    }

    void BaseTask::add_to(QpProblem& /*qp*/, const Reference& /*ref*/, const Pose2d& /*base_pose*/)
    {
        // TODO(M1)
    }

}  // namespace T1::wbc
