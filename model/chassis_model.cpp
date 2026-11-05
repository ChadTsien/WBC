#include "model/chassis_model.hpp"

namespace T1
{

    ChassisModel::ChassisModel(const ChassisModelConfig& config) : config_(config) {}

    BaseTwist ChassisModel::to_twist(const WheelSpeeds& /*wheels*/) const
    {
        // TODO(M0)
        return BaseTwist{};
    }

    WheelSpeeds ChassisModel::to_wheels(const BaseTwist& /*twist*/) const
    {
        // TODO(M0)
        return WheelSpeeds{};
    }

    Eigen::Matrix<double, 6, kNumChassisInputs> ChassisModel::point_jacobian(
        const Vec3& /*p_in_base*/, double /*yaw*/) const
    {
        // TODO(M1)
        return Eigen::Matrix<double, 6, kNumChassisInputs>::Zero();
    }

}  // namespace T1
