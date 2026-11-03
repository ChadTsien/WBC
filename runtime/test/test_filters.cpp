// LowPass1  /  Butterworth2
#include "runtime/filters.hpp"
#include "runtime/math.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace
{
    template <class Filter>
    double sine_amplitude(Filter& f, double fs, double freq_hz, int skip, int n)
    {
        double sum_sq = 0.0;
        for (int i = 0; i < skip + n; ++i)
        {
            const double x = 
                std::sin(2.0 * T1::kPi * freq_hz * static_cast<double>(i) / fs);
            const double y = f.update(x);
            if (i >= skip)
            {
                sum_sq += y * y;
            }
        }
        return std::sqrt(2.0 * sum_sq / static_cast<double>(n));
    }
}

TEST(Filters, DcGainIsOne)
{
    T1::LowPass1 lp;
    lp.configure(30.0, 500.0);
    T1::Butterworth2 bw;
    bw.configure(30.0, 500.0);

    double y_lp = 0.0;
    double y_bw = 0.0;
    for (int i = 0; i < 2000; ++i)
    {
        y_lp = lp.update(1.0);
        y_bw = bw.update(1.0);
    }
    EXPECT_NEAR(y_lp, 1.0, 1e-6);
    EXPECT_NEAR(y_bw, 1.0, 1e-6);
}

TEST(Filters, ButterworthCutoffAndStopband)
{
    T1::Butterworth2 fc;
    fc.configure(30.0, 500.0);
    const double a_cut = sine_amplitude(fc, 500.0, 30.0, 500, 1000);
    EXPECT_NEAR(a_cut, 0.707, 0.707 * 0.03);

    T1::Butterworth2 stop;
    stop.configure(30.0, 500.0);
    const double a_high = sine_amplitude(stop, 500.0, 150, 500, 1000);
    EXPECT_LT(a_high, 0.1);
}

TEST(Filters, ResetIsSteady)
{
    T1::LowPass1 lp;
    lp.configure(30.0, 500.0);
    lp.reset(5.0);
    EXPECT_DOUBLE_EQ(lp.update(5.0), 5.0);

    T1::Butterworth2 bw;
    bw.configure(30.0, 500.0);
    bw.reset(5.0);
    EXPECT_DOUBLE_EQ(bw.update(5.0), 5.0);
}