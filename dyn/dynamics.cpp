#include "dyn/dynamics.hpp"

namespace T1 {

Dynamics::Dynamics(RobotModel& model) : model_(model) {}

void Dynamics::set_tool_payload(double /*mass*/, const Vec3& /*com_in_ft*/) {
  // TODO(M3a)
}

ArmVec Dynamics::gravity_torque(const ArmVec& /*q*/, const Vec3& /*gravity_in_base*/) {
  // TODO(M3b)
  return ArmVec::Zero();
}

}  // namespace T1
