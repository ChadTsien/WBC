// RateLimiter
#include "runtime/rate_limiter.hpp"

#include <gtest/gtest.h>

TEST(RateLimiter, ReachesTargetInExactSteps)
{
    T1::RateLimiter rl;
    rl.configure(10.0);
    rl.reset(0.0);

    double y = 0.0;
    for (int i = 0; i < 9; ++i)
    {
        y = rl.update(1.0, 0.01);
        EXPECT_LT(y, 1.0);
    }
    // 测试钳位
    y = rl.update(1.0, 0.01);
    EXPECT_DOUBLE_EQ(y, 1.0);   
    EXPECT_DOUBLE_EQ(rl.update(1.0, 0.01), 1.0);
}

TEST(RateLimiter, DownwardIsSymmetric)
{
    T1::RateLimiter rl;
    rl.configure(10.0);
    rl.reset(1.0);

    for (int i = 0; i < 9; ++i)
    {
        EXPECT_GT(rl.update(0.0, 0.01), 0.0);
    }
    EXPECT_NEAR(rl.update(0.0, 0.01), 0.0, 1e-12);
}


TEST(RateLimiter, UnlimitedWhenRateNonPositive)
{
    T1::RateLimiter rl;
    rl.configure(0.0);
    rl.reset(0.0);
    EXPECT_DOUBLE_EQ(rl.update(5.0, 0.01), 5.0);
}