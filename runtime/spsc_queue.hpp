#pragma once

#include <array>
#include <atomic>
#include <cstddef>

namespace T1
{

    // Lock-free bounded queue: exactly one producer thread and one consumer thread.
    // Safe to call from a real-time thread (no allocation, no locks).
    // Holds at most Capacity - 1 elements.
    template <typename T, std::size_t Capacity>
    class SpscQueue
    {
        static_assert(Capacity >= 2, "Capacity must be at least 2");

        public:
        // Returns false when full; the element is dropped.
        bool push(const T& item)
        {
            // TODO(M0)
            (void)item;
            return false;
        }

        // Returns false when empty.
        bool pop(T& out)
        {
            // TODO(M0)
            (void)out;
            return false;
        }

        std::size_t size_approx() const
        {
            // TODO(M0)
            return 0;
        }

        private:
        std::array<T, Capacity> buffer_{};
        std::atomic<std::size_t> head_{0};  // next slot to read
        std::atomic<std::size_t> tail_{0};  // next slot to write
    };

}  // namespace T1
