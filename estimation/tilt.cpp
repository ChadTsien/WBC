#include "estimation/tilt.hpp"
#include "types/types.hpp"

namespace T1::est
{

    Vec3 gravity_in_base(const Quat& imu_orientation, const Mat3& base_R_imu, double g)
    {
        const Vec3 g_world(0.0, 0.0, -g);
        const Vec3 g_imu = imu_orientation.conjugate() * g_world;
        return base_R_imu * g_imu;
    }

}  // namespace T1::est
