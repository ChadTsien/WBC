#include "runtime/loop_timer.hpp"

#include <ctime>

#include <gtest/gtest.h>

TEST(PeriodicTimer, OneKhzForOneSecond)
{
    T1::PeriodicTimer t{0.001};
    t.start();

    timespec t0{};
    timespec t1{};
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (int i = 0; i < 1000; ++i)
    {
        t.wait();
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);

    const double elapsed =
        static_cast<double>(t1.tv_sec - t0.tv_sec)
        + static_cast<double>(t1.tv_nsec - t0.tv_nsec) * 1e-9;
    EXPECT_NEAR(elapsed, 1.0, 0.02);
    EXPECT_LT(t.overruns(), 20u);
}

TEST(LoopStats, PercentilesOnOneToHundred)
{
    T1::LoopStats s(100);
    for (int i = 1; i <= 100; ++i)
    {
        s.add(static_cast<double>(i));
    }
    EXPECT_EQ(s.count(), 100u);
    EXPECT_NEAR(s.percentile(50.0), 50.0, 1.0);
    EXPECT_NEAR(s.percentile(99.0), 99.0, 1.0);
    EXPECT_DOUBLE_EQ(s.max(), 100.0);
}

TEST(LoopStats, OverwriteOldest)
{
    T1::LoopStats s(3);
    s.add(1.0);
    s.add(2.0);
    s.add(3.0);
    s.add(10.0);
    EXPECT_EQ(s.count(), 3u);
    EXPECT_DOUBLE_EQ(s.max(), 10.0);
}