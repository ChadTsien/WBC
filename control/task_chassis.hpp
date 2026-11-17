#pragma once

#include "control/qp_problem.hpp"
#include "control/types.hpp"

namespace T1::wbc
{
    /**
     * 底盘任务参数
     *
     * 成员
     * - regularization：kFree 时 [v, ω] 的权重；底盘是慢自由度，宜小
     * - track_weight：kTrackTwist / kTrackPose 的权重，宜大
     * - kp_position：1/s，kTrackPose 前向误差 → v
     * - kp_yaw：1/s，kTrackPose 航向误差 → ω
     */
    struct ChassisTaskConfig
    {
        double regularization = 10.0;
        double track_weight = 1e4;
        double kp_position = 1.0;
        double kp_yaw = 1.5;
    };

    /**
     * 按 ChassisMode 给 [v, ω] 加代价
     *
     * 说明
     * - A 均为 2×8：[I₂ | 0]，只看底盘，不看关节
     * - kFree：b = 0，权重 regularization；车少动，臂够不着时由末端带动
     * - kTrackTwist：b = 参考 [v, ω]，权重 track_weight
     * - kTrackPose：车体系 e_b = Rz(−yaw)(p_ref − p)；
     *   v = kp_position · e_b.x，ω = kp_yaw · wrap_angle(yaw_ref − yaw)，
     *   再加上参考 [v, ω] 当前馈；丢掉 e_b.y（差速不能横移）
     */
    class ChassisTask
    {
        public:
            void configure(const ChassisTaskConfig& config);
            /**
             * 把本拍底盘代价写入 qp
             *
             * 参数
             * - ref：chassis_mode / chassis_pose / chassis_twist
             * - chassis_pose：当前底盘位姿，odom
             */
            void add_to(QpProblem& qp, const Reference& ref, const Pose2d& chassis_pose);
            
        private:
            ChassisTaskConfig config_;
    };
}  // namespace T1::wbc