// SpscQueue
#include "runtime/spsc_queue.hpp"

#include <cstdint>
#include <thread>

#include <gtest/gtest.h>

TEST(SpscQueue, SingleThreadFifo)
{
    T1::SpscQueue<int, 4> q;
    int out = 0;
    EXPECT_FALSE(q.pop(out));
    EXPECT_EQ(q.size_approx(), 0u);

    // 进队
    EXPECT_TRUE(q.push(1));
    EXPECT_TRUE(q.push(2));
    EXPECT_TRUE(q.push(3));
    EXPECT_FALSE(q.push(4));
    EXPECT_EQ(q.size_approx(), 3u);

    EXPECT_TRUE(q.pop(out));
    EXPECT_EQ(out, 1);
    EXPECT_TRUE(q.pop(out));
    EXPECT_EQ(out, 2);
    EXPECT_TRUE(q.pop(out));
    EXPECT_EQ(out, 3);
    EXPECT_FALSE(q.pop(out));
}

TEST(SpscQueue, TwoThreadsNoLoss)
{
    T1::SpscQueue<std::uint64_t, 1024> q;
    constexpr std::uint64_t n = 1000000;

    std::thread producer([&]() {
        for (std::uint64_t i = 0; i < n; ++i)
        {
            while (!q.push(i)) {
            
            }
        }
    });

    std::uint64_t prev = static_cast<std::uint64_t>(-1);
    std::uint64_t got = 0;
    std::uint64_t x = 0;
    while (got < n) 
    {
        if (q.pop(x)) 
        {
            EXPECT_EQ(x, prev + 1);
            prev = x;
            ++got;
        }
    }
    producer.join();
    EXPECT_EQ(got, n);
}