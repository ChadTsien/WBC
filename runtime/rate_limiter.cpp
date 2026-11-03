#include "runtime/rate_limiter.hpp"

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

    double RateLimiter::update(double target, double /*dt*/)
    {
        // TODO(M0)
        value_ = target;
        return value_;
    }

}  // namespace T1
