#include "control/qp_problem.hpp"
#include "control/qp_solver.hpp"
#include "control/types.hpp"
#include "runtime/loop_timer.hpp"
#include "types/types.hpp"

#include <gtest/gtest.h>

namespace 
{
    void add_identity_cost(T1::wbc::QpProblem& qp, const T1::WbcVec& b)
    {
        T1::MatX A = T1::MatX::Identity(T1::kNumWbcDof, T1::kNumWbcDof);
        qp.add_cost(A, b, 1.0);
    }
}

TEST(QpSolver, UnconstrainedMatchesMinusHinvG)
{
    T1::WbcVec b;
    b << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6, 0.7, -0.8;
    T1::wbc::QpProblem problem(0);
    add_identity_cost(problem, b);

    T1::wbc::QpSolver solver(T1::wbc::QpSolverConfig{}, 0);
    T1::WbcVec x = T1::WbcVec::Zero();
    int iter = 0;
    double us = 0.0;
    EXPECT_EQ(solver.solve(problem, &x, &iter, &us), T1::wbc::QpStatus::kOk);
    EXPECT_NEAR((x - b).norm(), 0.0, 1e-4);
}

TEST(QpSolver, BoxConstraintHitsBound)
{
    T1::WbcVec b = T1::WbcVec::Constant(1.0);
    T1::wbc::QpProblem problem(0);
    add_identity_cost(problem, b);
    T1::WbcVec lo = T1::WbcVec::Constant(-10.0);
    T1::WbcVec hi = T1::WbcVec::Constant(10.0);
    lo[0] = -0.1;
    hi[0] = 0.1;
    problem.intersect_bounds(lo, hi);

    T1::wbc::QpSolver solver(T1::wbc::QpSolverConfig{}, 0);
    T1::WbcVec x = T1::WbcVec::Zero();
    EXPECT_EQ(solver.solve(problem, &x, nullptr, nullptr), T1::wbc::QpStatus::kOk);
    EXPECT_NEAR(x[0], 0.1, 1e-4);
}

TEST(QpSolver, InfeasibleLeavesXUnchanged)
{
    T1::wbc::QpProblem problem(0);
    add_identity_cost(problem, T1::WbcVec::Ones());
    problem.intersect_bounds(T1::WbcVec::Constant(1.0), T1::WbcVec::Constant(-1.0));
    
    T1::wbc::QpSolver solver(T1::wbc::QpSolverConfig{}, 0);
    T1::WbcVec x = T1::WbcVec::Constant(3.0);
    EXPECT_EQ(solver.solve(problem, &x, nullptr, nullptr), T1::wbc::QpStatus::kInfeasible);
    EXPECT_DOUBLE_EQ(x[0], 3.0);
}

TEST(QpSolver, SolveP99Under200Us)
{
    T1::WbcVec b;
    b << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6, 0.7, -0.8;
    T1::wbc::QpProblem problem(0);
    add_identity_cost(problem, b);
    
    T1::wbc::QpSolver solver(T1::wbc::QpSolverConfig{}, 0);
    T1::LoopStats stats(1000);
    T1::WbcVec x = T1::WbcVec::Zero();
    for (int i = 0; i < 1000; ++i)
    {
        int iter = 0;
        double us = 0.0;
        ASSERT_EQ(solver.solve(problem, &x, &iter, &us), T1::wbc::QpStatus::kOk);
        stats.add(us);
    }
    EXPECT_LT(stats.percentile(99.0), 200.0);
}