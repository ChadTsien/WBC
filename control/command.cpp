#include "control/command.hpp"

namespace T1::wbc {

void CommandBuilder::configure(const CommandConfig& config) { config_ = config; }

void CommandBuilder::reset(const ArmVec& q_measured) { q_des_ = q_measured; }

MitCommand CommandBuilder::build(const WbcVec& /*x*/, const ArmVec& /*q_measured*/,
                                 const ArmVec& /*tau_gravity*/, bool /*contact_gains*/,
                                 double /*dt*/) {
  // TODO(M1)
  MitCommand cmd;
  cmd.q = q_des_;
  return cmd;
}

}  // namespace T1::wbc
