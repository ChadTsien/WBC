#include "model/chassis_model.hpp"
#include "types/types.hpp"

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