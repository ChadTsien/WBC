#pragma once

namespace T1
{

    /**
     * 按最大变化率逼近目标
     *
     * 递推（update），r = max_rate，单位 1/s
     * - r ≤ 0：不限速，y ← target
     * - 否则：Δ = clamp(target − y, −r·dt, +r·dt)，y ← y + Δ
     *
     * 说明
     * - dt 单位 s，须 > 0
     * - 一步走不完就停在目标上，不会越过
     * - 导纳 / 接触阶段的期望力斜坡用这个
     */
    class RateLimiter
    {
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
