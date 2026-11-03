#include "runtime/filters.hpp"
#include "runtime/math.hpp"

#include <cmath>

namespace T1
{
    void LowPass1::configure(double cutoff_hz, double sample_hz)
    {
        // RC = 1/(2π fc), dt = 1/fs, a = dt/(RC + dt)
        const double rc = 1.0 / (2.0 * kPi * cutoff_hz);
        const double dt = 1.0 / sample_hz;
        alpha_ = dt / (rc + dt);
    }

    void LowPass1::reset(double value)
    {
        y_ = value;
        initialized_ = true;
    }

    double LowPass1::update(double x)
    {
        if (!initialized_)
        {
            y_ = x;   // 第一次：用输入当初值
            initialized_ = true;
            return y_;
        }
        y_ += alpha_ * (x - y_);   // y ← y + α(x − y)
        return y_;
    }

    void Butterworth2::configure(double cutoff_hz, double sample_hz)
    {
        // K = tan(π fc / fs)
        // n = 1 / (1 + √2 K + K²)
        // a1 = 2(K² − 1) n, a2 = (1 − √2 K + K²) n
        // b0 = K² n, b1 = 2 b0, b2 = b0   
        const double K = std::tan(kPi * cutoff_hz / sample_hz);
        const double n = 1.0 / (1.0 + kSqrt2 * K + K * K);
        a1_ = 2.0 * (K * K - 1.0) * n;
        a2_ = (1.0 - std::sqrt(2.0) * K + K * K) * n;
        b0_ = K * K * n;
        b1_ = 2 * b0_;
        b2_ = b0_;
    }

    void Butterworth2::reset(double value)
    {
        x1_ = value;
        x2_ = value;
        y1_ = value;
        y2_ = value;
        initialized_ = true;
    }

    double Butterworth2::update(double x)
    {
        if (!initialized_)
        {
            reset(x);
            return x;
        }
        // y = b0 x + b1 x1 + b2 x2 − a1 y1 − a2 y2
        const double y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
        x2_ = x1_;
        x1_ = x;
        y2_ = y1_;
        y1_ = y;
        return y;
    }

}  // namespace T1
