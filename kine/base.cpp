#include "kine/base.hpp"

namespace T1 {

BaseKinematics::BaseKinematics(const DiffDriveParams& params) : params_(params) {}

BaseTwist BaseKinematics::to_twist(const WheelSpeeds& /*wheels*/) const {
  // TODO(M0)
  return BaseTwist{};
}

WheelSpeeds BaseKinematics::to_wheels(const BaseTwist& /*twist*/) const {
  // TODO(M0)
  return WheelSpeeds{};
}

Eigen::Matrix<double, 6, kNumBaseInputs> BaseKinematics::point_jacobian(const Vec3& /*p_in_base*/,
                                                                        double /*yaw*/) const {
  // TODO(M1)
  return Eigen::Matrix<double, 6, kNumBaseInputs>::Zero();
}

}  // namespace T1
