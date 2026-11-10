#include "sim/sensors.hpp"

#include <cmath>

#include "gtest/gtest.h"
#include "sim/world.hpp"
#include "types/types.hpp"

TEST(ImuModel, SameSeedReproducible)
{
    T1::sim::ImuNoiseConfig cfg;
    cfg.gyro_bias_std = 0.01;
    cfg.gyro_noise_std = 0.02;
    cfg.accel_noise_std = 0.05;
    T1::sim::ImuModel a(cfg, 42);
    T1::sim::ImuModel b(cfg, 42);
    const T1::Vec3 t(0.1, -0.2, 0.3);
    EXPECT_TRUE(a.apply_gyro(t).isApprox(b.apply_gyro(t)));
    EXPECT_TRUE(a.apply_accel(t).isApprox(b.apply_accel(t)));
}

TEST(ImuModel, GyroBiasWhenNoiseOff)
{
    T1::sim::ImuNoiseConfig cfg;
    cfg.gyro_noise_std = 0.0;
    cfg.gyro_bias_std = 0.01;
    T1::sim::ImuModel m(cfg, 1);
    const T1::Vec3 t = T1::Vec3::Zero();
    const T1::Vec3 y0 = m.apply_gyro(t);
    EXPECT_GT(y0.norm(), 1e-6);
    EXPECT_TRUE(m.apply_gyro(t).isApprox(y0));
}

TEST(ImuModel, GyroNoiseStats)
{
    T1::sim::ImuNoiseConfig cfg;
    cfg.gyro_bias_std = 0.0;
    cfg.gyro_noise_std = 0.02;
    T1::sim::ImuModel m(cfg, 7);
    const T1::Vec3 truth(0.1, -0.2, 0.3);
    const int n = 10000;
    T1::Vec3 sum = T1::Vec3::Zero();
    T1::Vec3 sumsq = T1::Vec3::Zero();
    for (int i = 0; i < n; ++i)
    {
        const T1::Vec3 y = m.apply_gyro(truth);
        sum += y;
        sumsq += y.cwiseAbs2();
    } 
    const T1::Vec3 mean = sum / n;
    const T1::Vec3 var = sumsq / n - mean.cwiseAbs2();
    EXPECT_TRUE(mean.isApprox(truth, 0.05));
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(std::sqrt(var[i]), cfg.gyro_noise_std, 0.05 * cfg.gyro_noise_std);
    }
}

TEST(ImuModel, AccelNoiseStats)
{
    T1::sim::ImuNoiseConfig cfg;
    cfg.accel_noise_std = 0.02;
    T1::sim::ImuModel m(cfg, 7);
    const T1::Vec3 truth(0.0, 0.0, 9.81);
    const int n = 10000;
    T1::Vec3 sum = T1::Vec3::Zero();
    T1::Vec3 sumsq = T1::Vec3::Zero();
    for (int i = 0; i < n; ++i)
    {
        const T1::Vec3 y = m.apply_accel(truth);
        sum += y;
        sumsq += y.cwiseAbs2();
    }
    const T1::Vec3 mean = sum / n;
    const T1::Vec3 var = sumsq / n - mean.cwiseAbs2();
    EXPECT_TRUE(mean.isApprox(truth, 0.05));
    for (int i = 0; i < 3; ++i)
    {
        EXPECT_NEAR(std::sqrt(var[i]), cfg.accel_noise_std, 0.05 * cfg.accel_noise_std);
    }
}