// 一阶低通、二阶巴特沃斯、向量 FilterBank. 每个实例只给一路信号用
#pragma once

#include <Eigen/Core>
#include <array>

namespace T1
{

    /**
     * 一阶 IIR 低通
     *
     * 系数（configure）
     * - RC = 1 / (2π fc)
     * - dt = 1 / fs
     * - α  = dt / (RC + dt)
     *
     * 递推（update）
     * - 第一次：y = x（用输入当初值）
     * - 之后：  y ← y + α (x − y)
     *
     * 说明
     * - 一路信号一个实例
     * - fc：截止频率 Hz；fs：采样频率 Hz
     */
    class LowPass1
    {
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

    /**
     * 二阶巴特沃斯低通（双线性变换，biquad）
     *
     * 系数（configure），K = tan(π fc / fs)
     * - norm = 1 / (1 + √2 K + K²)
     * - b0 = K² · norm
     * - b1 = 2 b0
     * - b2 = b0
     * - a1 = 2 (K² − 1) · norm
     * - a2 = (1 − √2 K + K²) · norm
     *
     * 递推（update），x1=x[n−1], x2=x[n−2], y1=y[n−1], y2=y[n−2]
     * - y[n] = b0 x[n] + b1 x1 + b2 x2 − a1 y1 − a2 y2
     * - 然后把延迟往后挪一拍
     *
     * 说明
     * - reset(v) 把 x1,x2,y1,y2 都设成 v，当作已经在直流稳态
     * - 第一次 update 若没 reset，用输入当 v，避免从 0 起步的尖峰
     */
    class Butterworth2
    {
        public:
        void configure(double cutoff_hz, double sample_hz);
        void reset(double value);
        double update(double x);

        private:
        double b0_ = 1.0, b1_ = 0.0, b2_ = 0.0, a1_ = 0.0, a2_ = 0.0;
        double x1_ = 0.0, x2_ = 0.0, y1_ = 0.0, y2_ = 0.0;
        bool initialized_ = false;
    };

    /**
     * N 路相同的标量滤波器，用力旋量这类向量
     *
     * 说明
     * - 每一维独立滤波，系数相同
     */
    template <class Filter, int N>
    class FilterBank
    {
        public:
            using Vec = Eigen::Matrix<double, N, 1>;

            void configure(double cutoff_hz, double sample_hz)
            {
                for (auto& f : filters_) f.configure(cutoff_hz, sample_hz);
            }
            void reset(const Vec& value)
            {
                for (int i = 0; i < N; ++i) filters_[i].reset(value[i]);
            }
            Vec update(const Vec& x)
            {
                Vec y;
                for (int i = 0; i < N; ++i) y[i] = filters_[i].update(x[i]);
                return y;
            }

        private:
            std::array<Filter, N> filters_;
    };

}  // namespace T1
