#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <vector>

namespace T1
{

    // Fixed-period loop on CLOCK_MONOTONIC with absolute deadlines (no drift).
    class PeriodicTimer
    {
        public:
        explicit PeriodicTimer(double period_s);
        void start();
        // Sleeps until the next deadline. Returns false if the deadline had already passed.
        bool wait();
        std::uint64_t overruns() const { return overruns_; }

        private:
        long period_ns_;
        timespec next_{};
        std::uint64_t overruns_ = 0;
    };

    // Rolling timing statistics. Memory is allocated in the constructor only,
    // so add() is safe in a real-time loop; percentiles are for the non-RT side.
    class LoopStats
    {
        public:
        explicit LoopStats(std::size_t window);
        void add(double sample_us);
        double percentile(double p) const;  // p in [0, 100]
        double max() const;
        std::size_t count() const { return count_; }
        void reset();

        private:
        std::vector<double> samples_;
        std::size_t next_ = 0;
        std::size_t count_ = 0;
    };

}  // namespace T1
