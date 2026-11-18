#include "estimation/tilt.hpp"
#include "runtime/math.hpp"
#include "types/types.hpp"

#include <cmath>
#include <gtest/gtest.h>

TEST(Tilt, IdentityIsDown)
{
    const T1::Vec3 g = T1::est::gravity_in_base(T1::Quat::Identity(), T1::Mat3::Identity());
    EXPECT_NEAR(g.x(), 0.0, 1e-12);
    EXPECT_NEAR(g.y(), 0.0, 1e-12);
    EXPECT_NEAR(g.z(), -9.81, 1e-12);
}

TEST(Tilt, RollRightGivesNegativeY)
{
    const double theta = 20.0 * T1::kPi / 180.0;
    const T1::Quat q(Eigen::AngleAxisd(theta, T1::Vec3::UnitX()));
    const T1::Vec3 g = T1::est::gravity_in_base(q, T1::Mat3::Identity());
    EXPECT_NEAR(g.x(), 0.0, 1e-12);
    EXPECT_NEAR(g.y(), -9.81 * std::sin(theta), 1e-12);
    EXPECT_NEAR(g.z(), -9.81 * std::cos(theta), 1e-12);
}

TEST(Tilt, YawDoesNotChangeGravity)
{
    const T1::Quat q(Eigen::AngleAxisd(1.7, T1::Vec3::UnitZ()));
    const T1::Vec3 g = T1::est::gravity_in_base(q, T1::Mat3::Identity());
    EXPECT_NEAR(g.x(), 0.0, 1e-12);
    EXPECT_NEAR(g.y(), 0.0, 1e-12);
    EXPECT_NEAR(g.z(), -9.81, 1e-12);
}
