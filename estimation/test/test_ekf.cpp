#include "estimation/ekf.hpp"
#include "runtime/math.hpp"
#include "types/types.hpp"

#include <cmath>
#include <random>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

namespace
{
    void expect_pd(const T1::est::PlanarEkf::Cov& P)
    {
        EXPECT_TRUE(P.isApprox(P.transpose(), 1e-10));
        const Eigen::SelfAdjointEigenSolver<T1::est::PlanarEkf::Cov> es(P);
        for (int i = 0; i < 5; ++i)
        {
            EXPECT_GT(es.eigenvalues()[i], 0.0);
        }
    }

    void step(T1::est::PlanarEkf* ekf, double dt, const T1::ChassisTwist& odom, double gyro)
    {
        ekf->predict(dt);
        ekf->update_odometry(odom);
        ekf->update_gyro(gyro);
    }
}  // namespace

TEST(Ekf, StationaryNoNoise)
{
    T1::est::PlanarEkf ekf;
    ekf.configure(T1::est::EkfConfig{});
    ekf.reset(T1::Pose2d{});
    const T1::ChassisTwist z{};
    for (int i = 0; i < 100; ++i)
    {
        step(&ekf, 0.002, z, 0.0);
    }
    EXPECT_NEAR(ekf.pose().x, 0.0, 1e-12);
    EXPECT_NEAR(ekf.pose().y, 0.0, 1e-12);
    EXPECT_NEAR(ekf.pose().yaw, 0.0, 1e-12);
    EXPECT_NEAR(ekf.twist().v, 0.0, 1e-12);
    EXPECT_NEAR(ekf.twist().omega, 0.0, 1e-12);
    expect_pd(ekf.covariance());
}

TEST(Ekf, StraightTenSeconds)
{
    T1::est::PlanarEkf ekf;
    ekf.configure(T1::est::EkfConfig{});
    ekf.reset(T1::Pose2d{});

    const double dt = 0.002;
    const double v = 0.5;
    const int n = static_cast<int>(10.0 / dt);
    std::mt19937 rng(1);
    std::normal_distribution<double> nv(0.0, std::sqrt(1e-4));
    std::normal_distribution<double> nw(0.0, std::sqrt(4e-6));
    for (int i = 0; i < n; ++i)
    {
        step(&ekf, dt, T1::ChassisTwist{v + nv(rng), nw(rng)}, nw(rng));
    }
    EXPECT_NEAR(ekf.twist().v, v, 0.01);
    EXPECT_NEAR(ekf.pose().y, 0.0, 0.05);
    expect_pd(ekf.covariance());
}

TEST(Ekf, SlipRejectsOdometryKeepsYaw)
{
    T1::est::PlanarEkf ekf;
    ekf.configure(T1::est::EkfConfig{});
    ekf.reset(T1::Pose2d{});

    const double dt = 0.002;
    const double v = 0.3;
    for (int i = 0; i < 500; ++i)
    {
        step(&ekf, dt, T1::ChassisTwist{v, 0.0}, 0.0);
    }
    const double yaw_before = ekf.pose().yaw;

    ekf.predict(dt);
    EXPECT_FALSE(ekf.update_odometry(T1::ChassisTwist{v, 1.0}));
    EXPECT_TRUE(ekf.update_gyro(0.0));
    EXPECT_NEAR(ekf.pose().yaw, yaw_before, 1e-3);
    expect_pd(ekf.covariance());
}
