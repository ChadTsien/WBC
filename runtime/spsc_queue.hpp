#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace T1
{

    /**
     * 无锁 SPSC 环形队列
     *
     * 流程
     * - push：t = tail；若 (t+1)%C == head 则满，返回 false
     *         否则写入 buffer[t]，再 tail ← (t+1)%C（release）
     * - pop： h = head；若 h == tail 则空，返回 false
     *         否则读出 buffer[h]，再 head ← (h+1)%C（release）
     *
     * 说明
     * - 恰好一个生产者、一个消费者；实时侧可调用（无分配、无锁）
     * - tail_ 只由生产者写，head_ 只由消费者写
     * - 写数据之后 store(release)；对端 load(acquire) 之后才读数据
     * - 最多容纳 Capacity − 1 个元素
     */
    template <typename T, std::size_t Capacity>
    class SpscQueue
    {
        static_assert(Capacity >= 2, "Capacity must be at least 2");

        public:
            bool push(const T& item)
            {
                const std::size_t t = tail_.load(std::memory_order_relaxed);
                const std::size_t next = (t + 1) % Capacity;
                if (next == head_.load(std::memory_order_acquire))
                {
                    return false;
                }
                buffer_[t] = item;
                tail_.store(next, std::memory_order_release);
                return true;
            }

            // Returns false when empty.
            bool pop(T& out)
            {
                const std::size_t h = head_.load(std::memory_order_relaxed);
                if (h == tail_.load(std::memory_order_acquire))
                {
                    return false;
                }
                out = buffer_[h];
                head_.store((h + 1) % Capacity, std::memory_order_relaxed);
                return true;
            }

            /**
             * 当前元素个数的估计值
             *
             * 说明
             * - (tail − head + C) % C，环形距离
             * - 两次 relaxed load 之间对端可能已拨指针，故只是近似
             * - 不要用它判断满/空；push/pop 自己看指针
             */
            std::size_t size_approx() const
            {
                const std::size_t h = head_.load(std::memory_order_relaxed);
                const std::size_t t = tail_.load(std::memory_order_relaxed);
                return (t + Capacity - h) % Capacity;
            }

        private:
            std::array<T, Capacity> buffer_{};
            std::atomic<std::size_t> head_{0};  // next slot to read
            std::atomic<std::size_t> tail_{0};  // next slot to write
    };

}  // namespace T1
