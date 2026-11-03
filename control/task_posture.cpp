#include "control/task_posture.hpp"

namespace T1::wbc
{

    void PostureTask::configure(const PostureTaskConfig& config)
    {
        config_ = config;
    }

    void PostureTask::add_to(QpProblem& /*qp*/, const ArmVec& /*q*/)
    {
        // TODO(M1)
    }

}  // namespace T1::wbc
