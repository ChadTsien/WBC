#include "estimation/odometry.hpp"
#include "model/chassis_model.hpp"
#include "runtime/math.hpp"
#include "types/types.hpp"

#include <gtest/gtest.h>

TEST(Odometry, StraightBothWheels)
{
    const T1::ChassisModelConfig c;
    const T1::ChassisModel chassis(c);
    T1::est::DiffDriveOdometry odom(chassis);
    odom.reset(T1::Pose2d{}, T1::WheelSpeeds{});

    const double theta = 1.0;
    const double dt = 0.1;
    odom.update(T1::WheelSpeeds{theta, theta}, dt);

    EXPECT_NEAR(odom.pose().x, c.wheel_radius * theta, 1e-12);
    EXPECT_NEAR(odom.pose().y, 0.0, 1e-12);
    EXPECT_NEAR(odom.pose().yaw, 0.0, 1e-12);
    EXPECT_NEAR(odom.twist().v, c.wheel_radius * theta / dt, 1e-12);
    EXPECT_NEAR(odom.twist().omega, 0.0, 1e-12);
}

TEST(Odometry, SpinInPlace)
{
    const T1::ChassisModelConfig c;
    const T1::ChassisModel chassis(c);
    T1::est::DiffDriveOdometry odom(chassis);
    odom.reset(T1::Pose2d{}, T1::WheelSpeeds{});

    const double theta = 0.5;
    const double dt = 0.2;
    odom.update(T1::WheelSpeeds{-theta, theta}, dt);

    const double dpsi = 2.0 * c.wheel_radius * theta / c.wheel_separation;
    EXPECT_NEAR(odom.pose().x, 0.0, 1e-12);
    EXPECT_NEAR(odom.pose().y, 0.0, 1e-12);
    EXPECT_NEAR(odom.pose().yaw, dpsi, 1e-12);
    EXPECT_NEAR(odom.twist().v, 0.0, 1e-12);
    EXPECT_NEAR(odom.twist().omega, dpsi / dt, 1e-12);
}

TEST(Odometry, CircleReturnsToStart)
{
    const T1::ChassisModel chassis(T1::ChassisModelConfig{});
    T1::est::DiffDriveOdometry odom(chassis);
    odom.reset(T1::Pose2d{}, T1::WheelSpeeds{});

    const double v = 0.4;
    const double omega = 0.5;
    const double T = 2.0 * T1::kPi / omega;
    const int n = 2000;
    const double dt = T / static_cast<double>(n);
    const T1::WheelSpeeds rate = chassis.to_wheels(T1::ChassisTwist{v, omega});

    T1::WheelSpeeds pos;
    for (int i = 0; i < n; ++i)
    {
        pos.left += rate.left * dt;
        pos.right += rate.right * dt;
        odom.update(pos, dt);
    }

    EXPECT_NEAR(odom.pose().x, 0.0, 1e-6);
    EXPECT_NEAR(odom.pose().y, 0.0, 1e-6);
    EXPECT_NEAR(odom.pose().yaw, 0.0, 1e-6);
}
