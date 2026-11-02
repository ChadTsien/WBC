#include "runtime/math.hpp"

namespace T1 {

double wrap_angle(double angle) {
  // TODO(M0)
  return angle;
}

Mat3 skew(const Vec3& /*v*/) {
  // TODO(M0)
  return Mat3::Zero();
}

Vec3 so3_log(const Mat3& /*rotation*/) {
  // TODO(M0)
  return Vec3::Zero();
}

double yaw_of(const Mat3& /*rotation*/) {
  // TODO(M0)
  return 0.0;
}

VecX saturate_norm(const VecX& v, double /*max_norm*/) {
  // TODO(M0)
  return v;
}

}  // namespace T1
