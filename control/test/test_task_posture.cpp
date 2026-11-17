#include "control/qp_problem.hpp"
#include "control/qp_solver.hpp"
#include "control/task_posture.hpp"
#include "types/types.hpp"

#include <gtest/gtest.h>

TEST(TaskPosture, ArmTracksBChassisStaysZero)
{
    T1::wbc::PostureTask task;
    T1::wbc::PostureTaskConfig cfg;
    cfg.kp = 2.0;
    cfg.q_nominal << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6;
    task.configure(cfg);

    const T1::ArmVec q = T1::ArmVec::Zero();
    const T1::ArmVec b = cfg.kp * (cfg.q_nominal - q);

    T1::wbc::QpProblem qp(0);
    task.add_to(qp, q);

    T1::MatX A_reg = T1::MatX::Zero(2, T1::kNumWbcDof);
    A_reg.leftCols<2>().setIdentity();
    qp.add_cost(A_reg, T1::VecX::Zero(2), 1.0);

    T1::wbc::QpSolver solver(T1::wbc::QpSolverConfig{}, 0);
    T1::WbcVec x = T1::WbcVec::Zero();
    
    EXPECT_EQ(solver.solve(qp, &x, nullptr, nullptr), T1::wbc::QpStatus::kOk);
    EXPECT_NEAR(x.head<2>().norm(), 0.0, 1e-4);
    EXPECT_NEAR((x.tail<6>() - b).norm(), 0.0, 1e-4);
}