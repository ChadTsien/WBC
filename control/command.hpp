#pragma once

#include "control/types.hpp"

namespace T1::wbc
{
    /**
     * MIT 增益一组
     *
     * 成员
     * - kp / kd：位置刚度、速度阻尼，进电机内部 PD
     */
    struct GainSet
    {
        ArmVec kp = ArmVec::Constant(30.0);
        ArmVec kd = ArmVec::Constant(1.0);
    };

    /**
     * CommandBuilder 参数
     *
     * 成员
     * - free_motion：空中运动增益
     * - contact：接触时更软的一组（M3b 才切过去）
     * - q_min / q_max：发出去的 q_des 硬限位
     * - max_tracking_error：q_des 相对实测 q 的最大偏离，rad 或 m
     */
    struct CommandConfig
    {
        GainSet free_motion;
        GainSet contact;
        ArmVec q_min = ArmVec::Constant(-3.0);
        ArmVec q_max = ArmVec::Constant(3.0);
        double max_tracking_error = 0.1;
    };

    /**
     * QP 速度解 → MIT 指令
     *
     * 说明
     * - q_des 从上一次 q_des 积分，不从实测 q 积分，电机看到的轨迹连续
     * - 再夹到 [q_meas ± max_tracking_error] 和 [q_min, q_max]
     * - dq_des = x 的关节部分；tau_ff = 重力矩
     * - kp/kd 由 contact_gains 在两组里选
     */
    class CommandBuilder
    {
        public:
            void configure(const CommandConfig& config);

            /**
            * 把内部 q_des 对齐到当前实测，使能 / 故障恢复时调用
            */
            void reset(const ArmVec& q_measured);

            /**
            * 积一分发出本拍 MIT 指令
            *
            * 参数
            * - x：本拍 QP 解 [v, ω, dq]
            * - q_measured：当前关节位置
            * - tau_gravity：重力前馈，直接写入 tau_ff
            * - contact_gains：true 用 contact，false 用 free_motion
            * - dt：控制周期，s
            */
            MitCommand build(const WbcVec& x, const ArmVec& q_measured, const ArmVec& tau_gravity,
                            bool contact_gains, double dt);

            /** 取出底盘 [v, ω]，给差速 to_wheels */
            static ChassisTwist chassis_twist(const WbcVec& x) { return ChassisTwist{x[0], x[1]}; }

        private:
            CommandConfig config_;
            ArmVec q_des_ = ArmVec::Zero();
    };

}  // namespace T1::wbc
