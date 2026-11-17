#include "control/command.hpp"
#include "types/types.hpp"

#include <gtest/gtest.h>

TEST(Command, ConstantDqIntegratesLinearly)
{
    T1::wbc::CommandBuilder b;
    b.configure(T1::wbc::CommandConfig{});
    const T1::ArmVec q0 = T1::ArmVec::Zero();
    b.reset(q0);

    T1::WbcVec x = T1::WbcVec::Zero();
    x.tail<6>().setConstant(0.5);
    const double dt = 0.01;
    T1::MitCommand cmd;
    for (int i = 0; i < 10; ++i)
    {
        cmd = b.build(x, q0, T1::ArmVec::Zero(), false, dt);
    }
    EXPECT_NEAR(cmd.q[0], 0.5 * dt * 10, 1e-12);
    EXPECT_NEAR(cmd.dq[0], 0.5, 1e-12);
}

TEST(Command, TrackingErrorClamp)
{
    T1::wbc::CommandBuilder b;
    T1::wbc::CommandConfig cfg;
    b.configure(cfg);
    b.reset(T1::ArmVec::Zero());

    T1::WbcVec x = T1::WbcVec::Zero();
    x.tail<6>().setConstant(10.0);
    const T1::ArmVec q_meas = T1::ArmVec::Zero();
    T1::MitCommand cmd;
    for (int i = 0; i < 50; ++i)
    {
        cmd = b.build(x, q_meas, T1::ArmVec::Zero(), false, 0.01);
    }
    EXPECT_NEAR(cmd.q[0], cfg.max_tracking_error, 1e-12);
}


TEST(Command, ResetSetsQdesToMeasured)
{
    T1::wbc::CommandBuilder b;
    b.configure(T1::wbc::CommandConfig{});
    b.reset(T1::ArmVec::Zero());

    T1::WbcVec x = T1::WbcVec::Zero();
    x.tail<6>().setConstant(1.0);
    b.build(x, T1::ArmVec::Zero(), T1::ArmVec::Zero(), false, 0.01);
    
    const T1::ArmVec q_meas = T1::ArmVec::Constant(0.3);
    b.reset(q_meas);
    const T1::MitCommand cmd =
        b.build(T1::WbcVec::Zero(), q_meas, T1::ArmVec::Zero(), false, 0.01);
    EXPECT_NEAR(cmd.q[0], 0.3, 1e-12);
}