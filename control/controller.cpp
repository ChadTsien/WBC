#include "control/controller.hpp"

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
        base_task_.configure(config.base);
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

    Output WbcController::update(const RobotState& state, const Reference& /*ref*/, double /*dt*/)
    {
        // TODO(M1)
        Output out;
        out.arm.q = state.arm.q;
        return out;
    }

    void WbcController::set_tool_payload(double mass, const Vec3& com_in_ft)
    {
        dynamics_.set_tool_payload(mass, com_in_ft);
    }

}  // namespace T1::wbc
