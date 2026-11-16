#pragma once

#include "control/types.hpp"

namespace T1::wbc
{

    /**
     * 按时间 t（相对轨迹起点，秒）插值出当前参考
     *
     * 说明
     * - 位置、速度、力线性插值；姿态 slerp；yaw 走短路径
     * - 离散量（底盘模式、接触阶段）取区间左端点
     * - t 早于首点：返回首点；晚于末点：末点位姿、速度为 0
     * - traj 非空且已按 t 排序；不分配内存
     */
    Reference sample_reference(const Trajectory& traj, double t);

}  // namespace T1::wbc
