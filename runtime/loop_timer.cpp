#include "runtime/loop_timer.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <ctime>

namespace  
{
    constexpr long kNsPerSec = 1000000000L;

    void add_ns(timespec& t, long ns)
    {
        t.tv_nsec += ns;
        while (t.tv_nsec >= kNsPerSec)
        {
            t.tv_nsec -= kNsPerSec;
            t.tv_sec += 1;
        }
    }

    bool earlier(const timespec& a, const timespec& b)
    {
        return a.tv_sec < b.tv_sec
               || (a.tv_sec == b.tv_sec && a.tv_nsec < b.tv_nsec);
    }
}

namespace T1
{

    PeriodicTimer::PeriodicTimer(double period_s) : period_ns_(static_cast<long>(period_s * 1e9)) {}

    void PeriodicTimer::start()
    {
        clock_gettime(CLOCK_MONOTONIC, &next_);
    }

    bool PeriodicTimer::wait()
    {
        add_ns(next_, period_ns_);
        timespec now{};
        clock_gettime(CLOCK_MONOTONIC, &now);

        // 当这这一拍已经晚了 
        if (!earlier(now, next_))
        {
            while (!earlier(now, next_))
            {
                ++overruns_;
                add_ns(next_, period_ns_);
            }
            return false;
        }
        while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_, nullptr)
               == EINTR)
        {
        }
        return true;
    }

    LoopStats::LoopStats(std::size_t window) : samples_(window, 0.0) {}

    void LoopStats::add(double sample_us)
    {
        if (samples_.empty())
        {
            return;
        }
        samples_[next_] = sample_us;
        next_ = (next_ + 1) % samples_.size();
        if (count_ < samples_.size())
        {
            ++count_;
        }
    }

    double LoopStats::percentile(double p) const
    {
        if (count_ == 0)
        {
            return 0.0;
        }
        p = std::clamp(p, 0.0, 100.0);
        std::vector<double> tmp(samples_.begin(), samples_.begin() + static_cast<std::ptrdiff_t>(count_));
        std::size_t idx = 0;
        if (p > 0.0)
        {
            idx = static_cast<std::size_t>(
                      std::ceil(p / 100.0 * static_cast<double>(count_)))
                  - 1;
            if (idx >= count_)
            {
                idx = count_ - 1;
            }
        }
        std::nth_element(tmp.begin(), tmp.begin() + static_cast<std::ptrdiff_t>(idx), tmp.end());
        return tmp[idx];
    }

    double LoopStats::max() const
    {
        if (count_ == 0)
        {
            return 0.0;
        }
        return *std::max_element(
            samples_.begin(), samples_.begin() + static_cast<std::ptrdiff_t>(count_));
    }

    void LoopStats::reset()
    {
        next_ = 0;
        count_ = 0;
    }

}  // namespace T1
