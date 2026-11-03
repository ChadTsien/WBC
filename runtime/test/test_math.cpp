// wrap_angle / skew / so3_log / yaw_of / saturate_norm
#include "runtime/math.hpp"

#include <algorithm>

#include <Eigen/src/Geometry/AngleAxis.h>
#include <gtest/gtest.h>

TEST(Math, WrapAngle)
{
    EXPECT_NEAR(T1::wrap_angle(1.5 * T1::kPi), -0.5 * T1::kPi, 1e-12);
    EXPECT_DOUBLE_EQ(T1::wrap_angle(-T1::kPi), T1::kPi);
    EXPECT_NEAR(T1::wrap_angle(7.0 * T1::kPi), T1::kPi, 1e-12);
}

TEST(Math, SkewCrossProduct)
{
    const T1::Vec3 a(1.0, -2.0, 3.0);
    const T1::Vec3 b(0.5, 4.0, -1.0);
    const T1::Vec3 got = T1::skew(a) * b;
    const T1::Vec3 want = a.cross(b);  // 叉乘
    EXPECT_NEAR((got - want).norm(), 0.0, 1e-12);

    for (int i = 0; i < 8; ++i)
    {
        const T1::Vec3 u(0.1 * i, 1.0 - 0.2 * i, 0.3 * i - 0.5);
        const T1::Vec3 v(-0.4 * i, 0.7, 2.0 - 0.1 * i);
        EXPECT_NEAR((T1::skew(u) * v - u.cross(v)).norm(), 0.0, 1e-12);
    }
}

TEST(Math, So3Log)
{
    const T1::Vec3 n = T1::Vec3(1.0, 2.0, -1.0).normalized();
    const T1::Mat3 R = Eigen::AngleAxisd(0.3, n).toRotationMatrix();
    EXPECT_NEAR((T1::so3_log(R) - 0.3 * n).norm(), 0.0, 1e-12);

    const T1::Vec3 log_pi = T1::so3_log(Eigen::AngleAxisd(T1::kPi, n).toRotationMatrix());
    const double e_plus = (log_pi - T1::kPi * n).norm();
    const double e_minus = (log_pi + T1::kPi * n).norm();
    EXPECT_LT(std::min(e_plus, e_minus), 1e-9);
}

TEST(Math, YawOf)
{
    const T1::Mat3 Ryaw = Eigen::AngleAxisd(0.4, T1::Vec3::UnitZ()).toRotationMatrix();
    EXPECT_NEAR(T1::yaw_of(Ryaw), 0.4, 1e-12);
    EXPECT_NEAR(T1::yaw_of(T1::Mat3::Identity()), 0.0, 1e-12);
}

TEST(Math, SaturateNorm)
{
    T1::VecX v(3);
    v << 3.0, 4.0, 0.0;   // 范数 5
    EXPECT_EQ(T1::saturate_norm(v, 10.0), v);

    const T1::VecX s = T1::saturate_norm(v, 2.0);
    EXPECT_NEAR(s.norm(), 2.0, 1e-12);
    EXPECT_NEAR((s.normalized() - v.normalized()).norm(), 0.0, 1e-12);
}