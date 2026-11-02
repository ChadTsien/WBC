#include "estimation/tilt.hpp"

namespace T1::est {

Vec3 gravity_in_base(const Quat& /*imu_orientation*/, const Mat3& /*base_R_imu*/, double g) {
  // TODO(M2)
  return Vec3(0.0, 0.0, -g);
}

}  // namespace T1::est
