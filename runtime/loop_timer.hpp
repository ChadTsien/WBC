#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <vector>

namespace T1
{

    /**
     * CLOCK_MONOTONIC 上的定周期节拍，绝对 deadline，不漂移
     *
     * 流程
     * - start：记下当前时刻为 next_
     * - wait：next_ ← next_ + period，再 clock_nanosleep(TIMER_ABSTIME)
     *         若此刻已过 deadline：overruns_ 按错过的周期累加，
     *         next_ 跳到「现在之后」的下一个周期点，不追赶，返回 false
     *
     * 说明
     * - period 单位 s；先 start() 再循环 wait()
     * - 返回 true 表示睡到了 deadline，false 表示已经超时
     */
    class PeriodicTimer
    {
        public:
            explicit PeriodicTimer(double period_s);
            void start();
            // Sleeps until the next deadline. Returns false if the deadline had already passed.
            bool wait();
            std::uint64_t overruns() const { return overruns_; }

        private:
            long period_ns_;
            timespec next_{};
            std::uint64_t overruns_ = 0;
    };

    /**
     * 滚动耗时统计
     *
     * 说明
     * - 构造时按 window 一次性分配；add() 只写环形缓冲，实时侧可调用
     * - percentile / max 会拷贝，只在非实时侧用
     * - p ∈ [0, 100]；窗口满后旧样本被覆盖
     */
    class LoopStats
    {
        public:
            /** window：环形缓冲容量；0 则 add 为空操作。只在这里分配 */
            explicit LoopStats(std::size_t window);

            /** 写入一个样本，单位 µs。覆盖最旧的；不分配 */
            void add(double sample_us);

            /**
             * 当前窗口的百分位，p ∈ [0, 100]
             *
             * 说明
             * - 会拷贝再 nth_element，不要在实时循环里调
             * - 尚无样本时返回 0
             */
            double percentile(double p) const;  // p in [0, 100]

            /** 窗口内最大样本，µs；无样本时 0。同样会扫一遍，非实时侧用 */
            double max() const;

            /** 当前有效样本数，≤ window */
            std::size_t count() const { return count_; }

            /** 清空计数，不释放缓冲 */
            void reset();

        private:
            std::vector<double> samples_;
            std::size_t next_ = 0;
            std::size_t count_ = 0;
    };

}  // namespace T1
