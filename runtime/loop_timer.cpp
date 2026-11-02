#include "runtime/loop_timer.hpp"

namespace T1 {

PeriodicTimer::PeriodicTimer(double period_s) : period_ns_(static_cast<long>(period_s * 1e9)) {}

void PeriodicTimer::start() {
  // TODO(M1)
}

bool PeriodicTimer::wait() {
  // TODO(M1)
  return true;
}

LoopStats::LoopStats(std::size_t window) : samples_(window, 0.0) {}

void LoopStats::add(double /*sample_us*/) {
  // TODO(M1)
}

double LoopStats::percentile(double /*p*/) const {
  // TODO(M1)
  return 0.0;
}

double LoopStats::max() const {
  // TODO(M1)
  return 0.0;
}

void LoopStats::reset() {
  next_ = 0;
  count_ = 0;
}

}  // namespace T1
