#include "signal/identify.hpp"

namespace T1::ft {

void GravityIdentifier::add(const CalibrationSample& sample) { samples_.push_back(sample); }

IdentificationResult GravityIdentifier::solve(double /*max_condition*/) const {
  // TODO(M3a)
  return IdentificationResult{};
}

IdentificationResult GravityIdentifier::solve_bias(const ToolGravityParams& /*known*/) const {
  // TODO(M3a)
  return IdentificationResult{};
}

}  // namespace T1::ft
