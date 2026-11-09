#include "model/chassis_model.hpp"
#include "types/types.hpp"

namespace T1
{

    ChassisModel::ChassisModel(const ChassisModelConfig& config) : config_(config) {}

    ChassisTwist ChassisModel::to_twist(const WheelSpeeds& wheels) const
    {
        const double r = config_.wheel_radius;
        const double L = config_.wheel_separation;
        ChassisTwist twist;
        twist.v = r * (wheels.left + wheels.right) / 2.0;
        twist.omega = r * (wheels.right - wheels.left) / L;
        return twist;
    }

    WheelSpeeds ChassisModel::to_wheels(const ChassisTwist& twist) const
    {
        const double r = config_.wheel_radius;
        const double L = config_.wheel_separation;
        WheelSpeeds wheels;
        wheels.left = (twist.v - twist.omega * L / 2.0) / r;
        wheels.right = (twist.v + twist.omega * L / 2.0) / r;
        return wheels;
    }

    Eigen::Matrix<double, 6, kNumChassisInputs> ChassisModel::point_jacobian(
        const Vec3& /*p_in_base*/, double /*yaw*/) const
    {
        // TODO(M1)
        return Eigen::Matrix<double, 6, kNumChassisInputs>::Zero();
    }

}  // namespace T1
