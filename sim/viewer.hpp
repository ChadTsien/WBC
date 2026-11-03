#pragma once

#include <atomic>
#include <thread>

namespace T1::sim
{

    class MujocoWorld;

    // Optional GLFW window that renders the world at ~60 Hz in its own thread.
    // Copies mjData under MujocoWorld's internal mutex (friend access); never steps physics.
    class SimViewer
    {
        public:
        explicit SimViewer(const MujocoWorld& world);
        ~SimViewer();
        void start();
        void stop();

        private:
        void run();

        const MujocoWorld& world_;
        std::thread thread_;
        std::atomic<bool> running_{false};
    };

}  // namespace T1::sim
