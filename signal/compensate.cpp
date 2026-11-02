#include "signal/compensate.hpp"

namespace T1::ft {

void FtCompensator::configure(const FtCompensatorConfig& config) {
  config_ = config;
  filter_.configure(config.cutoff_hz, config.sample_hz);
}

void FtCompensator::set_params(const ToolGravityParams& params) { params_ = params; }

Wrench FtCompensator::update(const Wrench& raw, const Vec3& /*gravity_in_ft*/,
                             const Vec3& /*com_accel_in_ft*/) {
  // TODO(M3a)
  return raw;
}

void FtCompensator::request_tare() {
  // TODO(M3a)
}

}  // namespace T1::ft
