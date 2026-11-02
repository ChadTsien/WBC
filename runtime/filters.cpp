#include "runtime/filters.hpp"

namespace T1 {

void LowPass1::configure(double /*cutoff_hz*/, double /*sample_hz*/) {
  // TODO(M0)
}

void LowPass1::reset(double value) {
  y_ = value;
  initialized_ = true;
}

double LowPass1::update(double x) {
  // TODO(M0)
  return x;
}

void Butterworth2::configure(double /*cutoff_hz*/, double /*sample_hz*/) {
  // TODO(M0)
}

void Butterworth2::reset(double /*value*/) {
  // TODO(M0)
}

double Butterworth2::update(double x) {
  // TODO(M0)
  return x;
}

}  // namespace T1
