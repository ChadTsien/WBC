#pragma once

#include "types/types.hpp"

namespace T1 {

struct DiffDriveParams {
  double wheel_radius = 0.0655;     // m
  double wheel_separation = 0.406;  // m
};

struct WheelSpeeds {
  double left = 0.0;   // rad/s, positive drives forward
  double right = 0.0;  // rad/s
};

// Differential drive. Using [v, omega] as the base inputs makes the no-sideways-motion
// constraint hold by construction.
class BaseKinematics {
 public:
  explicit BaseKinematics(const DiffDriveParams& params);

  BaseTwist to_twist(const WheelSpeeds& wheels) const;
  WheelSpeeds to_wheels(const BaseTwist& twist) const;

  // Maps [v, omega] to the 6D twist [linear; angular] of a point rigidly attached to the
  // base, world axes. p_in_base: the point in base_link; yaw: base heading in world.
  Eigen::Matrix<double, 6, kNumBaseInputs> point_jacobian(const Vec3& p_in_base,
                                                          double yaw) const;

 private:
  DiffDriveParams params_;
};

}  // namespace T1
