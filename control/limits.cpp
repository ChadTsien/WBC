#include "control/limits.hpp"

namespace T1::wbc {

void LimitConstraints::configure(const LimitsConfig& config) { config_ = config; }

void LimitConstraints::apply(QpProblem& /*qp*/, const ArmVec& /*q*/, const WbcVec& /*x_prev*/,
                             double /*dt*/) const {
  // TODO(M1)
}

}  // namespace T1::wbc
