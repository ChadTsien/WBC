#include "control/reference.hpp"
#include "control/types.hpp"
#include "runtime/math.hpp"
#include "types/types.hpp"

#include <cmath>
#include <gtest/gtest.h>

namespace  
{
    T1::wbc::Trajectory two_points()
    {
        T1::wbc::TimedReference a;
        a.t = 0.0;
        a.ref.chassis_pose.x = 0.0;
        a.ref.chassis_pose.y = 0.0;
        a.ref.chassis_twist.v = 0.0;
        a.ref.ee_pose.translation() = T1::Vec3(2.0, 0.0, 0.0);
        a.ref.ee_twist.linear = T1::Vec3(0.0, 0.0, 0.0);
        a.ref.desired_normal_force = 0.0;
        a.ref.chassis_mode = T1::wbc::ChassisMode::kTrackTwist;

        T1::wbc::TimedReference b = a;
        b.t = 2.0;
        b.ref.chassis_pose.x = 2.0;
        b.ref.chassis_pose.y = 4.0;
        b.ref.chassis_twist.v = 1.0;
        b.ref.ee_pose.translation() = T1::Vec3(2.0, 0.0, 0.0);
        b.ref.ee_twist.linear = T1::Vec3(1.0, 0.0, 0.0);
        b.ref.desired_normal_force = 10.0;
        b.ref.chassis_mode = T1::wbc::ChassisMode::kTrackTwist;
        return {a, b};
    }
}

TEST(Reference, MidpointInterpolates)
{
    const auto traj = two_points();
    const T1::wbc::Reference r = T1::wbc::sample_reference(traj, 1.0);
    EXPECT_NEAR(r.chassis_pose.x, 1.0, 1e-12);
    EXPECT_NEAR(r.chassis_pose.y, 2.0, 1e-12);
    EXPECT_NEAR(r.chassis_twist.v, 0.5, 1e-12);
    EXPECT_NEAR(r.ee_pose.translation().x(), 2.0, 1e-12);
    EXPECT_NEAR(r.desired_normal_force, 5.0, 1e-12);
    EXPECT_EQ(r.chassis_mode, T1::wbc::ChassisMode::kTrackTwist);
}

TEST(Reference, BeforeFirstReturnsFirst)
{
    const auto traj = two_points();
    const T1::wbc::Reference r = T1::wbc::sample_reference(traj, -1.0);
    EXPECT_NEAR(r.chassis_pose.x, 0.0, 1e-12);
    EXPECT_NEAR(r.chassis_twist.v, 0.0, 1e-12);
}

TEST(Reference, AfterLastZerosVelocity)
{
    const auto traj = two_points();
    const T1::wbc::Reference r = T1::wbc::sample_reference(traj, 3.0);
    EXPECT_NEAR(r.chassis_pose.x, 2.0, 1e-12);
    EXPECT_NEAR(r.chassis_twist.v, 0.0, 1e-12);
    EXPECT_NEAR(r.ee_twist.linear.norm(), 0.0, 1e-12);
}

TEST(Reference, YawTakesShortPath)
{
    T1::wbc::TimedReference a;
    a.t = 0.0;
    a.ref.chassis_pose.yaw = 3.1;
    T1::wbc::TimedReference b;
    b.t = 1.0;
    b.ref.chassis_pose.yaw = -3.1;
    const T1::wbc::Reference r = T1::wbc::sample_reference({a, b}, 0.5);
    EXPECT_NEAR(std::abs(r.chassis_pose.yaw), T1::kPi, 0.05);
    EXPECT_GT(std::abs(r.chassis_pose.yaw), 3.0);
}