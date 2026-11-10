#include "sim/sensors.hpp"
#include <random>
#include "types/types.hpp"

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
        std::normal_distribution<double> n(0.0, config_.gyro_bias_std);
        gyro_bias_ = Vec3(n(rng_), n(rng_), n(rng_));
    }

    Vec3 ImuModel::apply_gyro(const Vec3& truth)
    {
        std::normal_distribution<double> n(0.0, config_.gyro_noise_std);
        return truth + gyro_bias_ + Vec3(n(rng_), n(rng_), n(rng_));
    }

    Vec3 ImuModel::apply_accel(const Vec3& truth)
    {
        std::normal_distribution<double> n(0.0, config_.accel_noise_std);
        return truth + Vec3(n(rng_), n(rng_), n(rng_));
    }

}  // namespace T1::sim
