#pragma once

#include "control/config.hpp"
#include "control/qp_problem.hpp"
#include "dyn/dynamics.hpp"
#include "kine/arm.hpp"
#include "model/robot_model.hpp"

namespace T1::wbc
{

    // One control cycle, ROS-free and allocation-free after construction:
    //   kinematics -> contact phase + admittance (modify reference) -> tasks & limits -> QP
    //   -> MIT command + base twist + wheel speeds
    class WbcController
    {
        public:
        WbcController(const WbcConfig& config, RobotModel& model);

        // Call on activation: holds the current pose, clears integrators and warm start.
        void reset(const RobotState& state);
        Output update(const RobotState& state, const Reference& ref, double dt);

        // Call from the control thread between update() calls (shares the model). No allocation.
        void set_tool_payload(double mass, const Vec3& com_in_ft);

        private:
        WbcConfig config_;
        ArmKinematics kinematics_;
        Dynamics dynamics_;
        BaseKinematics base_kinematics_;

        EeTrackingTask ee_task_;
        PostureTask posture_task_;
        BaseTask base_task_;
        LimitConstraints limits_;
        ContactPhaseManager contact_;
        Admittance admittance_;
        QpProblem qp_;
        QpSolver solver_;
        CommandBuilder command_;

        WbcVec x_prev_ = WbcVec::Zero();
    };

}  // namespace T1::wbc
