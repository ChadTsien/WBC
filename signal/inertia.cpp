#include "signal/inertia.hpp"

namespace T1::ft
{

    Wrench remove_inertia(const Wrench& wrench, const ToolGravityParams& /*params*/,
                          const Vec3& /*com_accel_in_ft*/)
    {
        // TODO(M3a)
        return wrench;
    }

}  // namespace T1::ft
