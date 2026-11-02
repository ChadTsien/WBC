#pragma once

namespace T1 {

// Moves the output toward a target with bounded rate of change (units per second).
class RateLimiter {
 public:
  void configure(double max_rate_per_s);
  void reset(double value);
  double update(double target, double dt);
  double value() const { return value_; }

 private:
  double max_rate_ = 0.0;
  double value_ = 0.0;
};

}  // namespace T1
