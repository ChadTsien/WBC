#pragma once

#include <string>

namespace T1
{
    /**
     * 当前线程的实时设置
     *
     * 成员
     * - cpu：绑到哪颗核；-1 不绑
     * - fifo_priority：0 保持 SCHED_OTHER；1..99 切 SCHED_FIFO
     * - lock_memory：true 则 mlockall，避免缺页进实时循环
     */
    struct RtConfig
    {
        int cpu = -1;              // -1: no pinning
        int fifo_priority = 0;     // 0: keep SCHED_OTHER; 1..99: SCHED_FIFO
        bool lock_memory = false;  // mlockall(MCL_CURRENT | MCL_FUTURE)
    };

    /**
     * 把 RtConfig 作用到调用线程
     *
     * 流程
     * - cpu ≥ 0：pthread_setaffinity_np
     * - fifo_priority ∈ [1, 99]：pthread_setschedparam(SCHED_FIFO)
     * - lock_memory：mlockall(MCL_CURRENT | MCL_FUTURE)
     *
     * 说明
     * - 任一步失败返回 false，error 写原因，并提示查 limits.conf 的 rtprio / memlock
     * - error 可为 nullptr
     * - 只改当前线程，不创建新线程
     */
    bool apply_rt_config(const RtConfig& config, std::string* error);

}  // namespace T1
