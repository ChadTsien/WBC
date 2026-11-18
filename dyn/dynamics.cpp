#include "dyn/dynamics.hpp"

#include <pinocchio/algorithm/rnea.hpp>

namespace T1
{

    Dynamics::Dynamics(ArmModel& model) : model_(model) {}

    void Dynamics::set_tool_payload(double /*mass*/, const Vec3& /*com_in_ft*/)
    {
        // TODO(M3a)
    }

    ArmVec Dynamics::gravity_torque(const ArmVec& q, const Vec3& gravity_in_base)
    {
        if (!q.allFinite() || !gravity_in_base.allFinite())
        {
            return ArmVec::Zero();
        }

        pinocchio::Model& model = model_.model();
        model.gravity.linear() = gravity_in_base;
        pinocchio::computeGeneralizedGravity(model, model_.data(), q);

        ArmVec tau;
        tau = model_.data().g;
        return tau;
    }

}  // namespace T1
