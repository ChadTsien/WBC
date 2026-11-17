#include "control/limits.hpp"
#include "control/qp_problem.hpp"
#include "types/types.hpp"

#include <gtest/gtest.h>

namespace  
{
    T1::wbc::LimitConstraints make_limits()
    {
        T1::wbc::LimitsConfig cfg;
        cfg.q_min = T1::ArmVec::Constant(-1.0);
        cfg.q_max = T1::ArmVec::Constant(1.0);
        cfg.position_margin = 0.05;
        cfg.position_gain = 5.0;
        T1::wbc::LimitConstraints lim;
        lim.configure(cfg);
        return lim;
    }
}

TEST(Limits, NearQmaxUpperBoundGoesToZero)
{
    auto lim = make_limits();
    T1::wbc::QpProblem qp(0);
    T1::ArmVec q = T1::ArmVec::Constant(0.95);
    lim.apply(qp, q, T1::WbcVec::Zero(), 0.002);
    EXPECT_NEAR(qp.x_upper()[2], 0.0, 1e-12);
}

TEST(Limits, PastQmaxUpperBoundNegativeAndFeasible)
{
    auto lim = make_limits();
    T1::wbc::QpProblem qp(0);
    T1::ArmVec q = T1::ArmVec::Constant(1.2);
    lim.apply(qp, q, T1::WbcVec::Zero(), 0.002);
    EXPECT_LT(qp.x_upper()[2], 0.0);
    EXPECT_LE(qp.x_lower()[2], qp.x_upper()[2]);
}

TEST(Limits, AccelBoundsCenteredOnXprev)
{
    auto lim = make_limits();
    T1::wbc::QpProblem qp(0);
    T1::WbcVec x_prev = T1::WbcVec::Zero();
    x_prev[0] = 0.2;
    const double dt = 0.01;
    lim.apply(qp, T1::ArmVec::Zero(), x_prev, dt);
    EXPECT_NEAR(qp.x_lower()[0], 0.2 - 0.5 * dt, 1e-12);
    EXPECT_NEAR(qp.x_upper()[0], 0.2 + 0.5 * dt, 1e-12);
}