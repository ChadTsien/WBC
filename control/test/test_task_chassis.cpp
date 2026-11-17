#include "control/qp_problem.hpp"
#include "control/qp_solver.hpp"
#include "control/task_chassis.hpp"
#include "control/types.hpp"
#include "runtime/math.hpp"
#include "types/types.hpp"

#include <gtest/gtest.h>


namespace
{
    T1::WbcVec solve_chassis(T1::wbc::ChassisTask& task, const T1::wbc::Reference& ref,
                             const T1::Pose2d& pose)
    {
        T1::wbc::QpProblem qp(0);
        task.add_to(qp, ref, pose);

        T1::MatX A_arm = T1::MatX::Zero(T1::kNumArmJoints, T1::kNumWbcDof);
        A_arm.rightCols<T1::kNumArmJoints>().setIdentity();
        qp.add_cost(A_arm, T1::VecX::Zero(T1::kNumArmJoints), 1.0);
        
        T1::wbc::QpSolver solver(T1::wbc::QpSolverConfig{}, 0);
        T1::WbcVec x = T1::WbcVec::Zero();

        EXPECT_EQ(solver.solve(qp, &x, nullptr, nullptr), T1::wbc::QpStatus::kOk);
        return x;
    }
}

TEST(TaskChassis, FreeGivesZeroTwist)
{
    T1::wbc::ChassisTask task;
    task.configure({});
    T1::wbc::Reference ref;
    ref.chassis_mode = T1::wbc::ChassisMode::kFree;
    const T1::WbcVec x = solve_chassis(task, ref, {});
    EXPECT_NEAR(x[0], 0.0, 1e-4);
    EXPECT_NEAR(x[1], 0.0, 1e-4);
}

TEST(TaskChassis, TrackPoseForwardGivesPositiveV)
{
    T1::wbc::ChassisTask task;
    task.configure({});
    T1::wbc::Reference ref;
    ref.chassis_mode = T1::wbc::ChassisMode::kTrackPose;
    ref.chassis_pose.x = 1.0;
    T1::Pose2d pose;
    const T1::WbcVec x = solve_chassis(task, ref, pose);
    EXPECT_GT(x[0], 0.0);
    EXPECT_NEAR(x[1], 0.0, 1e-4);
}

TEST(TaskChassis, TrackPoseYawErrorGivesPositiveOmega)
{
    T1::wbc::ChassisTask task;
    task.configure({});
    T1::wbc::Reference ref;
    ref.chassis_mode = T1::wbc::ChassisMode::kTrackPose;
    ref.chassis_pose.yaw = 0.5 * T1::kPi;
    T1::Pose2d pose;
    const T1::WbcVec x = solve_chassis(task, ref, pose);
    EXPECT_NEAR(x[0], 0.0, 1e-4);
    EXPECT_GT(x[1], 0.0);
}