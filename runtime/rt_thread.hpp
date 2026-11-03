#pragma once

#include <string>

namespace T1
{

    struct RtConfig
    {
        int cpu = -1;              // -1: no pinning
        int fifo_priority = 0;     // 0: keep SCHED_OTHER; 1..99: SCHED_FIFO
        bool lock_memory = false;  // mlockall(MCL_CURRENT | MCL_FUTURE)
    };

    // Applies the config to the calling thread. On failure returns false and fills error;
    // typical cause is missing rtprio / memlock limits in /etc/security/limits.conf.
    bool apply_rt_config(const RtConfig& config, std::string* error);

}  // namespace T1
