#include "runtime/rt_thread.hpp"

#include <string>

#include <gtest/gtest.h>

TEST(RtThread, PinCpuZeroWithoutFifo)
{
    T1::RtConfig cfg;
    cfg.cpu = 0;
    cfg.fifo_priority = 0;
    std::string err = "stale";
    EXPECT_TRUE(T1::apply_rt_config(cfg, &err)) << err;
    EXPECT_TRUE(err.empty());
}

TEST(RtThread, HighFifoFailsWithoutPermission)
{
    T1::RtConfig cfg;
    cfg.fifo_priority = 80;
    std::string err;
    if (T1::apply_rt_config(cfg, &err))
    {
        GTEST_SKIP() << "process has rtprio; SCHED_FIFO 80 succeeded";
    }
    EXPECT_FALSE(err.empty());
}