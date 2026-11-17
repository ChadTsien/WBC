#pragma once

#include "control/qp_problem.hpp"

namespace T1::wbc
{

    /**
     * 决策量 x 的盒约束参数
     *
     * 成员
     * - q_min / q_max：关节位置限位，rad 或 m（J3）
     * - dq_max：关节速度上限（对称）
     * - ddq_max：关节加速度上限（对称）
     * - v_max / omega_max：底盘速度上限
     * - dv_max / domega_max：底盘加速度上限
     * - position_margin：位置软限位相对硬限位内收，rad 或 m
     * - position_gain：1/s，靠近限位时允许的接近速度
     */
    struct LimitsConfig
    {
        ArmVec q_min = ArmVec::Constant(-3.0);
        ArmVec q_max = ArmVec::Constant(3.0);
        ArmVec dq_max = ArmVec::Constant(2.0);
        ArmVec ddq_max = ArmVec::Constant(10.0);
        double v_max = 0.5, omega_max = 1.0;
        double dv_max = 0.5, domega_max = 1.5;
        double position_margin = 0.05;
        double position_gain = 5.0;
    };

    /**
     * 把速度、加速度、位置限位写成 x 的盒子
     *
     * 说明
     * - 三次 intersect_bounds：速度、加速度、位置，取交集
     * - 速度：|v|≤v_max，|ω|≤omega_max，|dq|≤dq_max
     * - 加速度：夹在 x_prev ± a_max·dt
     * - 位置：dq ≤ gain (q_max − margin − q)，下界对称
     * - 若交集为空（已超出限位），放宽位置约束，只允许往限位内走
     */
    class LimitConstraints
    {
        public:
            void configure(const LimitsConfig& config);

            /**
             * 把本拍盒约束写入 qp
             *
             * 参数
             * - q：当前关节位置
             * - x_prev：上一拍 QP 解，加速度限位的中心
             * - dt：控制周期，s
             */
            void apply(QpProblem& qp, const ArmVec& q, const WbcVec& x_prev,
                       double dt) const;

        private:
            LimitsConfig config_;
    };

}  // namespace T1::wbc