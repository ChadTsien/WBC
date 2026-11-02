#include "control/admittance.hpp"

namespace T1::wbc {

void Admittance::configure(const AdmittanceConfig& config) { config_ = config; }

void Admittance::reset() {
  x_ = 0.0;
  v_ = 0.0;
  offset_.setZero();
  tilt_.setZero();
}

Vec3 Admittance::update_normal(const Vec3& /*normal*/, double /*measured_force*/,
                               double /*desired_force*/, double /*dt*/) {
  // TODO(M3b)
  return offset_;
}

Vec3 Admittance::update_tilt(const Vec3& /*torque_world*/, const Vec3& /*brush_axis*/,
                             double /*dt*/) {
  // TODO(M3b)
  return tilt_;
}

}  // namespace T1::wbc
