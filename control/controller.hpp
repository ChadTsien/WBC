#pragma once

#include "control/config.hpp"
#include "control/qp_problem.hpp"
#include "dyn/dynamics.hpp"
#include "kine/arm.hpp"
#include "model/arm_model.hpp"

namespace T1::wbc
{

    /**
     * 一个控制周期：运动学 → 任务与限位 → QP → MIT + 轮速
     *
     * 说明
     * - 构造后不再分配
     * - M1 不改参考、不用导纳；contact_gains 固定 false
     */
    class WbcController
    {
        public:
            WbcController(const WbcConfig& config, ArmModel& model);

            /** 使能时调用：积分器、接触、导纳清零，q_des 对齐实测 */
            void reset(const RobotState& state);

            /**
             * 算一拍指令
             *
             * 参数
             * - state：关节、底盘位姿、重力
             * - ref：工具位姿 / 底盘模式，odom
             * - dt：控制周期，s
             */
            Output update(const RobotState& state, const Reference& ref, double dt);

            /** 换工具时改负载；与 update 不同时调，共用 ArmModel */
            void set_tool_payload(double mass, const Vec3& com_in_ft);

            /**
             * 当前构型的重力矩，给 write_hold / MIT tau_ff
             *
             * 说明
             * - 与 update 里用的是同一套 Dynamics
             */
            ArmVec gravity_torque(const ArmVec& q, const Vec3& gravity_in_base);

        private:
            WbcConfig config_;
            ArmKinematics kinematics_;
            Dynamics dynamics_;
            ChassisModel chassis_;

            EeTrackingTask ee_task_;
            PostureTask posture_task_;
            ChassisTask chassis_task_;
            LimitConstraints limits_;
            ContactPhaseManager contact_;
            Admittance admittance_;
            QpProblem qp_;
            QpSolver solver_;
            CommandBuilder command_;

            WbcVec x_prev_ = WbcVec::Zero();
    };

}  // namespace T1::wbc
