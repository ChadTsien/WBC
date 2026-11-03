#include "control/task_ee.hpp"

namespace T1::wbc
{

    void EeTrackingTask::configure(const EeTaskConfig& config)
    {
        config_ = config;
    }

    void EeTrackingTask::add_to(QpProblem& /*qp*/, const WholeBodyJacobian& /*J*/,
                                const Iso3& /*ee_world*/, const Iso3& /*ee_ref*/,
                                const Twist& /*ee_twist_ref*/)
    {
        // TODO(M1)
    }

}  // namespace T1::wbc
