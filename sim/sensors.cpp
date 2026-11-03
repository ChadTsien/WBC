#include "sim/sensors.hpp"

namespace T1::sim
{

    FtSensorModel::FtSensorModel(const FtNoiseConfig& config, unsigned seed)
        : config_(config), rng_(seed)
    {
    }

    Wrench FtSensorModel::apply(const Wrench& truth, double /*dt*/)
    {
        // TODO(M3a)
        return truth;
    }

    ImuModel::ImuModel(const ImuNoiseConfig& config, unsigned seed) : config_(config), rng_(seed)
    {
        // TODO(M2): draw gyro_bias_
    }

    Vec3 ImuModel::apply_gyro(const Vec3& truth)
    {
        // TODO(M2)
        return truth;
    }

    Vec3 ImuModel::apply_accel(const Vec3& truth)
    {
        // TODO(M2)
        return truth;
    }

}  // namespace T1::sim
