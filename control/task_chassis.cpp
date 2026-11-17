#include "control/task_chassis.hpp"
#include "control/types.hpp"
#include "model/chassis_model.hpp"
#include "types/types.hpp"
#include "runtime/math.hpp"

#include <cmath>
#include <Eigen/src/Core/Matrix.h>

namespace T1::wbc
{

    void ChassisTask::configure(const ChassisTaskConfig& config)
    {
        config_ = config;
    }

    void ChassisTask::add_to(QpProblem& qp, const Reference& ref,
                            const Pose2d& chassis_pose)
    {
        MatX A = MatX::Zero(2, kNumWbcDof);
        A.leftCols<2>().setIdentity();

        Eigen::Vector2d b = Eigen::Vector2d::Zero();
        double weight = config_.regularization;

        switch (ref.chassis_mode)
        {
            case ChassisMode::kFree:
                break;

            case ChassisMode::kTrackTwist:
                b << ref.chassis_twist.v, ref.chassis_twist.omega;
                weight = config_.track_weight;
                break;

            case ChassisMode::kTrackPose:
            {
                const double c = std::cos(chassis_pose.yaw);
                const double s = std::sin(chassis_pose.yaw);
                const double dx = ref.chassis_pose.x - chassis_pose.x;
                const double dy = ref.chassis_pose.y - chassis_pose.y;
                const double e_forward = c * dx + s * dy;
                const double e_yaw =
                    wrap_angle(ref.chassis_pose.yaw - chassis_pose.yaw);
                b(0) = config_.kp_position * e_forward + ref.chassis_twist.v;
                b(1) = config_.kp_yaw * e_yaw + ref.chassis_twist.omega;
                weight = config_.track_weight;
                break;
            }
        }

        qp.add_cost(A, b, weight);
    }

}  // namespace T1::wbc
