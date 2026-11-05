#pragma once

#include "types/types.hpp"

namespace T1
{

    /**
     * 差速底盘参数
     *
     * 成员
     * - wheel_radius：轮半径 r，m
     * - wheel_separation：轮距 L，m
     */
    struct ChassisModelConfig
    {
        double wheel_radius = 0.0655;
        double wheel_separation = 0.406;
    };

    struct WheelSpeeds
    {
        double left = 0.0;   // rad/s, positive drives forward
        double right = 0.0;  // rad/s
    };

    /**
     * 差速底盘模型
     *
     * 说明
     * - 决策变量是 [v, ω]，不能横移由构造保证
     * - 臂在 ArmModel；全身拼装才是 RobotModel（kine/whole_body）
     */
    class ChassisModel
    {
        public:
        explicit ChassisModel(const ChassisModelConfig& config);

        BaseTwist to_twist(const WheelSpeeds& wheels) const;
        WheelSpeeds to_wheels(const BaseTwist& twist) const;

        /**
         * [v, ω] → 固连在底盘上一点的 6 维速度（世界系坐标轴）
         *
         * 说明
         * - p_in_base：该点在 base_link 下
         * - yaw：底盘在世界系的朝向
         */
        Eigen::Matrix<double, 6, kNumChassisInputs> point_jacobian(const Vec3& p_in_base,
                                                                   double yaw) const;

        private:
        ChassisModelConfig config_;
    };

}  // namespace T1
