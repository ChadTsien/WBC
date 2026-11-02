#include "signal/gravity.hpp"

namespace T1::ft {

Wrench gravity_wrench(const ToolGravityParams& /*params*/, const Vec3& /*gravity_in_ft*/) {
  // TODO(M3a)
  return Wrench{};
}

Wrench remove_gravity_and_bias(const Wrench& raw, const ToolGravityParams& /*params*/,
                               const Vec3& /*gravity_in_ft*/) {
  // TODO(M3a)
  return raw;
}

}  // namespace T1::ft
