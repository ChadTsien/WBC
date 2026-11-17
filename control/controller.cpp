#include "control/controller.hpp"

#include <Eigen/Geometry>

#include "control/types.hpp"
#include "kine/whole_body.hpp"
#include "types/types.hpp"

namespace T1::wbc
{

    WbcController::WbcController(const WbcConfig& config, ArmModel& model)
        : config_(config),
          kinematics_(model),
          dynamics_(model),
          chassis_(config.chassis),
          qp_(config.max_inequality_rows),
          solver_(config.qp, config.max_inequality_rows)
    {
        ee_task_.configure(config.ee);
        posture_task_.configure(config.posture);
        chassis_task_.configure(config.chassis_task);
        limits_.configure(config.limits);
        contact_.configure(config.contact);
        admittance_.configure(config.admittance);
        command_.configure(config.command);
    }

    void WbcController::reset(const RobotState& state)
    {
        x_prev_.setZero();
        contact_.reset();
        admittance_.reset();
        command_.reset(state.arm.q);
    }

    Output WbcController::update(const RobotState& state, const Reference& ref, double dt)
    {
        // 运动学更新
        kinematics_.update(state.arm.q);
        const Iso3 T_chassis_tool = kinematics_.tool_pose();

        // 求取工具在世界系中的表示
        Iso3 T_odom_chassis = Iso3::Identity();
        T_odom_chassis.translation() =
            Vec3(state.chassis_pose.x, state.chassis_pose.y, 0.0);
        T_odom_chassis.linear() =
            Eigen::AngleAxisd(state.chassis_pose.yaw, Vec3::UnitZ()).toRotationMatrix();
        const Iso3 T_odom_tool = T_odom_chassis * T_chassis_tool;

        // 全身雅可比
        const WholeBodyJacobian J = whole_body_jacobian(
            chassis_, state.chassis_pose, T_chassis_tool, kinematics_.tool_jacobian());
        
        /*  QP 任务添加  */
        qp_.reset();
        ee_task_.add_to(qp_, J, T_odom_tool, ref.ee_pose, ref.ee_twist);
        posture_task_.add_to(qp_, state.arm.q);
        chassis_task_.add_to(qp_, ref, state.chassis_pose);
        limits_.apply(qp_, state.arm.q, x_prev_, dt);

        Output out;
        WbcVec x = WbcVec::Zero();
        out.qp_status = solver_.solve(qp_, &x, &out.qp_iterations, &out.qp_solve_time_us);
        
        // 求解失败
        if (out.qp_status != QpStatus::kOk)
        {
            x.setZero();
        }
        x_prev_ = x;

        const ArmVec tau_g = dynamics_.gravity_torque(state.arm.q, state.gravity_in_base);
        out.arm = command_.build(x, state.arm.q, tau_g, false, dt);
        out.chassis = CommandBuilder::chassis_twist(x);
        out.wheels = chassis_.to_wheels(out.chassis);
        out.qp_solution = x;
        out.ee_error = ee_task_.last_error();
        out.manipulability = kinematics_.manipulability();
        return out;
    }

    void WbcController::set_tool_payload(double mass, const Vec3& com_in_ft)
    {
        dynamics_.set_tool_payload(mass, com_in_ft);
    }

}  // namespace T1::wbc
