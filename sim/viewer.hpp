#pragma once

#include <atomic>
#include <thread>

namespace T1::sim
{

    class MujocoWorld;

    /**
     * 可选 GLFW 窗口，独立线程约 60 Hz 渲染
     *
     * 说明
     * - 不推进物理；锁内拷/读 mjData，锁外画
     * - 窗口必须在 run() 线程里创建（GLFW 约束）
     */
    class SimViewer
    {
        public:
            explicit SimViewer(const MujocoWorld& world);
            ~SimViewer();
            void start();
            void stop();
            bool alive() const { return running_; }

        private:
            void run();

            const MujocoWorld& world_;
            std::thread thread_;
            std::atomic<bool> running_{false};
    };

}  // namespace T1::sim
