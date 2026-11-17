#pragma once

#include "control/qp_problem.hpp"

namespace T1::wbc
{
    /**
     * 姿态任务参数
     *
     * 成员
     * - q_nominal：舒适构型，一般取关节行程中位
     * - weight：按关节软优先级，默认小，只在末端零空间里起作用
     * - kp：1/s，b = kp (q_nominal − q)
     */
    struct PostureTaskConfig
    {
        ArmVec q_nominal = ArmVec::Zero();      // comfortable configuration, mid-range
        ArmVec weight = ArmVec::Constant(1.0);  // low: only acts in the task null space
        double kp = 1.0;                        // 1/s
    };

    /**
     * 把机械臂拉向 q_nominal
     *
     * 说明
     * - 代价：weight · ‖A x − b‖²，A 为 6×8
     * - A = [0₆ₓ₂ | I₆]：只约束 dq，不约束底盘
     * - b = kp (q_nominal − q)
     * - 目标够不着时，末端要走、姿态要收臂，底盘会跟着动
     */
    class PostureTask
    {
        public:
            void configure(const PostureTaskConfig& config);

            /**
             * 把本拍姿态代价写入 qp
             *
             * 参数
             * - q：当前关节角，与 ArmVec 下标相同
             */
            void add_to(QpProblem& qp, const ArmVec& q);

        private:
            PostureTaskConfig config_;
    };

}  // namespace T1::wbc
