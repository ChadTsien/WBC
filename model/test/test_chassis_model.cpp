#include "model/chassis_model.hpp"
#include "types/types.hpp"
#include "runtime/math.hpp"

#include <cmath>

#include <gtest/gtest.h>

TEST(ChassisModel, RoundTrip)
{
    const T1::ChassisModel chassis(T1::ChassisModelConfig{});
    T1::WheelSpeeds w;
    w.left = 1.2;
    w.right = 0.4;
    const T1::ChassisTwist t = chassis.to_twist(w); 
    const T1::WheelSpeeds back = chassis.to_wheels(t);
    EXPECT_NEAR(back.left, w.left, 1e-12);
    EXPECT_NEAR(back.right, w.right, 1e-12);
}

TEST(ChassisModel, SpinInPlaceHasZeroV)
{
    const T1::ChassisModel chassis(T1::ChassisModelConfig{});
    T1::WheelSpeeds w;
    w.left = -2.0;
    w.right = 2.0;
    const T1::ChassisTwist t = chassis.to_twist(w);
    EXPECT_NEAR(t.v, 0.0, 1e-12);
    EXPECT_GT(t.omega, 0.0);
}

TEST(ChassisModel, StraightHasZeroOmega)
{
    const T1::ChassisModel chassis(T1::ChassisModelConfig{});
    T1::WheelSpeeds w;
    w.left = 1.5;
    w.right = 1.5;
    const T1::ChassisTwist t = chassis.to_twist(w);
    EXPECT_NEAR(t.omega, 0.0, 1e-12);
    EXPECT_NEAR(t.v, 1.5 * 0.0655, 1e-12);
}

TEST(ChassisModel, PointJacobianMatchesFiniteDifference)
{
    const T1::ChassisModel chassis(T1::ChassisModelConfig{});
    const T1::Vec3 p_b(0.2, -0.1, 0.3);
    const double yaw = 0.4;
    const double v = 0.5;
    const double omega = -0.8;
    const double dt = 1e-6;

    // 底盘速度、角速度为世界系
    Eigen::Matrix<double, 2, 1> u;
    u << v, omega;
    const Eigen::Matrix<double, 6, 1> dx = chassis.point_jacobian(p_b, yaw) * u;

    const auto Rz = [](double y) {
        return Eigen::Matrix3d(Eigen::AngleAxisd(y, T1::Vec3::UnitZ()));
    };
    const T1::Vec3 p0 = Rz(yaw) * p_b;
    const T1::Vec3 p1 =
        T1::Vec3(v * std::cos(yaw) * dt, v * std::sin(yaw) * dt, 0.0)
        + Rz(yaw + omega * dt) * p_b;
    const T1::Vec3 dv = (p1 - p0) / dt;
    const T1::Vec3 dw = T1::so3_log(Rz(yaw + omega * dt) * Rz(yaw).transpose()) / dt;
    
    EXPECT_NEAR(dv.x(), dx(0), 1e-5);
    EXPECT_NEAR(dv.y(), dx(1), 1e-5);
    EXPECT_NEAR(dv.z(), dx(2), 1e-5);
    EXPECT_NEAR(dw.x(), dx(3), 1e-5);
    EXPECT_NEAR(dw.y(), dx(4), 1e-5);
    EXPECT_NEAR(dw.z(), dx(5), 1e-5);
}