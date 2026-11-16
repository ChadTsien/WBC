#include "control/qp_problem.hpp"

#include <gtest/gtest.h>

TEST(QpProblem, UnconstrainedOptimumIsMinusHinvG)
{
    T1::wbc::QpProblem qp(0);
    T1::MatX A = T1::MatX::Identity(T1::kNumWbcDof, T1::kNumWbcDof);
    T1::WbcVec b;
    b << 0.1, -0.2, 0.3, 0.4, -0.5, 0.6, 0.7, -0.8;
    qp.add_cost(A, b, 1.0);
    const T1::WbcVec x = -qp.H().ldlt().solve(qp.g());
    EXPECT_NEAR((x - b).norm(), 0.0, 1e-5);
}

TEST(QpProblem, InequalityOverflowRejected)
{
    T1::wbc::QpProblem qp(2);
    T1::MatX C = T1::MatX::Ones(2, T1::kNumWbcDof);
    T1::VecX lo = T1::VecX::Constant(2, -1.0);
    T1::VecX hi = T1::VecX::Constant(2, 1.0);
    EXPECT_TRUE(qp.add_inequality(C, lo, hi));
    EXPECT_EQ(qp.inequality_rows(), 2);

    T1::MatX C1 = T1::MatX::Ones(1, T1::kNumWbcDof);
    EXPECT_FALSE(qp.add_inequality(C1, T1::VecX::Zero(1), T1::VecX::Ones(1)));
    EXPECT_EQ(qp.inequality_rows(), 2);
    EXPECT_DOUBLE_EQ(qp.C()(0, 0), 1.0);
}

TEST(QpProblem, IntersectBoundsTakesTighterBox)
{
    T1::wbc::QpProblem qp(0);
    qp.intersect_bounds(T1::WbcVec::Constant(-10.0), T1::WbcVec::Constant(10.0));
    qp.intersect_bounds(T1::WbcVec::Constant(-1.0), T1::WbcVec::Constant(20.0));
    EXPECT_DOUBLE_EQ(qp.x_lower()[0], -1.0);
    EXPECT_DOUBLE_EQ(qp.x_upper()[0], 10.0);
}
