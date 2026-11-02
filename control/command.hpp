#pragma once

#include "control/types.hpp"

namespace T1::wbc {

struct GainSet {
  ArmVec kp = ArmVec::Constant(30.0);
  ArmVec kd = ArmVec::Constant(1.0);
};

struct CommandConfig {
  GainSet free_motion;
  GainSet contact;              // softer, used while the brush touches the surface
  ArmVec q_min = ArmVec::Constant(-3.0);
  ArmVec q_max = ArmVec::Constant(3.0);
  double max_tracking_error = 0.1;  // rad / m; q_des is pulled back if it runs away from q
};

// QP velocity -> MIT command. q_des integrates from the previous q_des (not from the
// measured q) so the motor PD sees a consistent setpoint; it is clamped to stay within
// max_tracking_error of the measured q.
class CommandBuilder {
 public:
  void configure(const CommandConfig& config);
  void reset(const ArmVec& q_measured);

  MitCommand build(const WbcVec& x, const ArmVec& q_measured, const ArmVec& tau_gravity,
                   bool contact_gains, double dt);
  static BaseTwist base_twist(const WbcVec& x) { return BaseTwist{x[0], x[1]}; }

 private:
  CommandConfig config_;
  ArmVec q_des_ = ArmVec::Zero();
};

}  // namespace T1::wbc
