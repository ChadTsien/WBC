#pragma once

#include <array>

#include <Eigen/Core>

namespace T1 {

// First-order IIR low pass. Not thread safe; one instance per signal.
class LowPass1 {
 public:
  void configure(double cutoff_hz, double sample_hz);
  void reset(double value);
  double update(double x);
  double value() const { return y_; }

 private:
  double alpha_ = 1.0;
  double y_ = 0.0;
  bool initialized_ = false;
};

// Second-order Butterworth low pass (biquad, bilinear transform).
class Butterworth2 {
 public:
  void configure(double cutoff_hz, double sample_hz);
  void reset(double value);
  double update(double x);

 private:
  double b0_ = 1.0, b1_ = 0.0, b2_ = 0.0, a1_ = 0.0, a2_ = 0.0;
  double x1_ = 0.0, x2_ = 0.0, y1_ = 0.0, y2_ = 0.0;
  bool initialized_ = false;
};

// N independent copies of a scalar filter, for vectors such as a wrench.
template <class Filter, int N>
class FilterBank {
 public:
  using Vec = Eigen::Matrix<double, N, 1>;

  void configure(double cutoff_hz, double sample_hz) {
    for (auto& f : filters_) f.configure(cutoff_hz, sample_hz);
  }
  void reset(const Vec& value) {
    for (int i = 0; i < N; ++i) filters_[i].reset(value[i]);
  }
  Vec update(const Vec& x) {
    Vec y;
    for (int i = 0; i < N; ++i) y[i] = filters_[i].update(x[i]);
    return y;
  }

 private:
  std::array<Filter, N> filters_;
};

}  // namespace T1
