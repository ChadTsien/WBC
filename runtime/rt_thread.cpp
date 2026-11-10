#include "runtime/rt_thread.hpp"
#include <string>

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <cerrno>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

namespace
{
    bool fail(std::string* error, const char* what, int err)
    {
        if (error)
        {
            *error = std::string(what) + ": " + std::strerror(err)
                     + "; check /etc/security/limits.conf for rtprio and memlock";
        }
        return false;
    }
}

namespace T1
{
    bool apply_rt_config(const RtConfig& config, std::string* error)
    {
        if (config.cpu >= 0)
        {
            cpu_set_t set;
            CPU_ZERO(&set);              // 清空集合
            CPU_SET(config.cpu, &set);   // 只留下 config.cpu 这一颗核
            const int rc =   //  绑核
                pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
            if (rc != 0)
            {
                return fail(error, "pthread_setaffinity_np", rc);
            }
        }

        // 将当前线程改成 SCHED_FIFO 实时调度
        if (config.fifo_priority > 0)
        {
            if (config.fifo_priority > 99)
            {
                return fail(error, "fifo_priority", EINVAL);
            }
            sched_param sp{};
            sp.sched_priority = config.fifo_priority;
            const int rc =
                pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
            if (rc != 0)
            {
                return fail(error, "pthread_setschedparam(SCHED_FIFO)", rc);
            }
        }

        // 内存锁定
        if (config.lock_memory)
        {
            if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
            {
                return fail(error, "mlockall", errno);
            }
        }

        if (error)
        {
            error->clear();
        }
        return true;
    }

}  // namespace T1
