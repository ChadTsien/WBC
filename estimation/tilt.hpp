#pragma once

#include "types/types.hpp"

namespace T1::est {

// Gravity vector [m/s^2] expressed in base_link, from the IMU orientation.
// imu_orientation: IMU frame in a gravity-aligned frame; base_R_imu: IMU mounting rotation.
Vec3 gravity_in_base(const Quat& imu_orientation, const Mat3& base_R_imu, double g = 9.81);

}  // namespace T1::est
