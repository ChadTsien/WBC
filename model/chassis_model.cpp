#include "model/chassis_model.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
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
        const Vec3& p_in_base, double yaw) const
    {
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);
        const Vec3 p_w(c * p_in_base.x() - s * p_in_base.y(),
                       s * p_in_base.x() + c * p_in_base.y(),
                       p_in_base.z());

        Eigen::Matrix<double, 6, kNumChassisInputs> j;
        j.setZero();
        j(0, 0) = c;
        j(1, 0) = s;
        j(0, 1) = -p_w.y();
        j(1, 1) = p_w.x();
        j(5, 1) = 1.0;
        return j;
    }

}  // namespace T1
