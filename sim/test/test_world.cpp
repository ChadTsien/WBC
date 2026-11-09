#include "sim/world.hpp"
#include "types/types.hpp"

#include "gtest/gtest.h"

namespace
{
    T1::sim::WorldConfig make_cfg()
    {
        T1::sim::WorldConfig cfg;
        cfg.mjcf_path = T1_TEST_MJCF;
        cfg.timestep = 0.001;
        return cfg;
    }
}

TEST(MujocoWorld, GravitySagsArmChassisStill)
{
    T1::sim::MujocoWorld world(make_cfg());
    world.reset();
    const T1::ArmVec q0 = world.arm_state().q;
    for (int i = 0; i < 1000; ++i)
    {
        world.step();
    }
    const T1::ArmVec q1 = world.arm_state().q;
    EXPECT_GT((q1 - q0).norm(), 1e-3);
    const T1::sim::GroundTruth gt = world.ground_truth();
    EXPECT_NEAR(gt.base_pose.x, 0.0, 5e-3);
    EXPECT_NEAR(gt.base_pose.y, 0.0, 5e-3);
    EXPECT_NEAR(gt.base_pose.yaw, 0.0, 5e-3);
}

TEST(MujocoWorld, Joint1TracksSetpoint)
{
    T1::sim::WorldConfig cfg = make_cfg();
    cfg.effort_limits << 35.0, 35.0, 200.0, 12.0, 12.0, 12.0;
    T1::sim::MujocoWorld world(cfg);
    world.reset();
    
    T1::MitCommand cmd;
    cmd.q[0] = 0.3;
    cmd.kp << 20.0, 80.0, 5000.0, 80.0, 80.0, 80.0;
    cmd.kd << 8.0, 5.0, 200.0, 2.0, 2.0, 2.0;
    world.set_arm_command(cmd);
    for (int i = 0; i < 2000; ++i)
    {
        world.step();
    }
    EXPECT_NEAR(world.arm_state().q[0], 0.3, 0.05);
    EXPECT_NEAR(world.arm_state().q[2], 0.0, 0.02);  // J3 没掉下去
}

TEST(MujocoWorld, EqualWheelsDriveForward)
{
    T1::sim::MujocoWorld world(make_cfg());
    world.reset();
    T1::WheelSpeeds w;
    w.left = 5.0;
    w.right = 5.0;
    world.set_wheel_command(w);
    for (int i = 0; i < 3000; ++i)
    {
        world.step();
    }
    const T1::sim::GroundTruth gt = world.ground_truth();
    EXPECT_GT(gt.base_pose.x, 0.9);
    EXPECT_NEAR(gt.base_pose.yaw, 0.0, 0.1);
}