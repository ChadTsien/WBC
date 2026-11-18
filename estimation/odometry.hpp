#pragma once

#include "model/chassis_model.hpp"
#include "types/types.hpp"

namespace T1::est
{

    /**
     * 差速轮编码器 → odom 系位姿和 [v, ω]
     *
     * 说明
     * - 用累计转角的差分，不用轮速
     * - 由 WbcRosController 在 500 Hz 调用，不单独起节点
     */
    class DiffDriveOdometry
    {
        public:
            explicit DiffDriveOdometry(const ChassisModel& chassis);

            /**
             * 把位姿和编码器零点对齐；使能时调用
             *
             * 参数
             * - pose：当前 odom 位姿
             * - wheel_positions：左右轮累计转角，rad
             */
            void reset(const Pose2d& pose, const WheelSpeeds& wheel_positions);
            
            /**
             * 积一分
             *
             * 参数
             * - wheel_positions：左右轮累计转角，rad（不是 rad/s）
             * - dt：周期，s，须 > 0
             */
            void update(const WheelSpeeds& wheel_positions, double dt);

            Pose2d pose() const { return pose_; }
            ChassisTwist twist() const { return twist_; }

        private:
            ChassisModel chassis_;
            Pose2d pose_;
            ChassisTwist twist_;
            WheelSpeeds last_positions_;
    };

}  // namespace T1::est
