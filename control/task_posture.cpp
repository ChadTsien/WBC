#include "control/task_posture.hpp"
#include "types/types.hpp"

namespace T1::wbc
{

    void PostureTask::configure(const PostureTaskConfig& config)
    {
        config_ = config;
    }

    void PostureTask::add_to(QpProblem& qp, const ArmVec& q)
    {
        MatX A = MatX::Zero(kNumArmJoints, kNumWbcDof);
        A.rightCols<kNumArmJoints>().setIdentity();
        const ArmVec b = config_.kp * (config_.q_nominal - q);
        qp.add_cost(A, b, VecX(config_.weight));
    }

}  // namespace T1::wbc
