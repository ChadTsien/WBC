#pragma once

#include "control/qp_problem.hpp"
#include "kine/whole_body.hpp"

namespace T1::wbc
{
    /**
     * 工具跟踪任务参数
     *
     * 成员
     * - kp：1/s，误差反馈；前 3 位姿、后 2 轴线
     * - weight：按行软优先级，越大越优先
     * - max_error_rate：kp·e 的范数上限；位置 m/s、轴线 rad/s 分开限
     */
    struct EeTaskConfig
    {
        Vec5 kp = Vec5::Constant(5.0);      // 1/s, error feedback gain
        Vec5 weight = Vec5::Constant(1e3);  // per task row
        double max_error_rate = 0.5;        // m/s (and rad/s) cap on kp * error
    };

    /**
     * 工具 5 维跟踪：位置 xyz + 轴线指向（不约束绕工具 z 的滚转）
     *
     * 说明
     * - 代价：weight · ‖A x − b‖²，A 为 5×8
     * - A：J 前 3 行；R.col(0)ᵀ J_ω；R.col(1)ᵀ J_ω（R 用当前 ee_world）
     * - b = v_ref + saturate(kp ⊙ e)；轴线误差 e_w = a × a_ref，再投到工具 x、y
     */
    class EeTrackingTask
    {
        public:
            void configure(const EeTaskConfig& config);

            /**
             * 把本拍末端代价写入 qp
             *
             * 参数
             * - J：whole_body_jacobian()，世界系坐标轴，6×8
             * - ee_world / ee_ref：brush_tip 在 odom
             * - ee_twist_ref：参考空间速度，世界系
             */
            void add_to(QpProblem& qp, const WholeBodyJacobian& J, const Iso3& ee_world,
                        const Iso3& ee_ref, const Twist& ee_twist_ref);
            
            /** 上一拍 5 维误差：位置 m，轴线 rad */
            const Vec5& last_error() const { return error_; }

        private:
            EeTaskConfig config_;
            Vec5 error_ = Vec5::Zero();
    };

}  // namespace T1::wbc
