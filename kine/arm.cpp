#include "kine/arm.hpp"

namespace T1 {

ArmKinematics::ArmKinematics(RobotModel& model) : model_(model) {}

void ArmKinematics::update(const ArmVec& /*q*/) {
  // TODO(M0)
}

Iso3 ArmKinematics::frame_pose(int /*frame_id*/) const {
  // TODO(M0)
  return Iso3::Identity();
}

Iso3 ArmKinematics::tool_pose() const { return frame_pose(model_.tool_frame_id()); }

Iso3 ArmKinematics::ft_pose() const { return frame_pose(model_.ft_frame_id()); }

ArmJacobian ArmKinematics::frame_jacobian(int /*frame_id*/) const {
  // TODO(M0)
  return ArmJacobian::Zero();
}

ArmJacobian ArmKinematics::tool_jacobian() const { return frame_jacobian(model_.tool_frame_id()); }

double ArmKinematics::manipulability() const {
  // TODO(M1)
  return 0.0;
}

}  // namespace T1
