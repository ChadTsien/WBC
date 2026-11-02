#pragma once

#include "types/types.hpp"

namespace T1 {

// Result in (-pi, pi].
double wrap_angle(double angle);

Mat3 skew(const Vec3& v);

// Rotation vector (axis * angle) of R, angle in [0, pi].
Vec3 so3_log(const Mat3& rotation);

// Yaw of the rotation's x axis projected onto the world XY plane.
double yaw_of(const Mat3& rotation);

// Scales v down so that its norm does not exceed max_norm. max_norm >= 0.
VecX saturate_norm(const VecX& v, double max_norm);

}  // namespace T1
