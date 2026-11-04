#include "runtime/rate_limiter.hpp"

#include <algorithm>

namespace T1
{

    void RateLimiter::configure(double max_rate_per_s)
    {
        max_rate_ = max_rate_per_s;
    }

    void RateLimiter::reset(double value)
    {
        value_ = value;
    }

    double RateLimiter::update(double target, double dt)
    {
        if (max_rate_ <= 0.0)
        {
            value_ = target;
            return value_;
        }
        const double max_step = max_rate_ * dt;
        const double delta = std::clamp(target - value_, -max_step, max_step);
        value_ += delta;
        return value_;
    }

}  // namespace T1
