#include "kine/arm.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/fwd.hpp>
#include "types/types.hpp"

namespace T1
{

    ArmKinematics::ArmKinematics(ArmModel& model) : model_(model) {}

    void ArmKinematics::update(const ArmVec& q)
    {
        const pinocchio::Model& model = model_.model();
        pinocchio::Data& data = model_.data();
        // 关节位姿 -> 各 frame 的 oMf -> 关节雅可比: getFrameJacobian 依赖后两步
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::computeJointJacobians(model, data);
    }

    Iso3 ArmKinematics::frame_pose(int frame_id) const
    {
        // 取出 frame 的位姿态
        const pinocchio::SE3& M = 
            model_.data().oMf[static_cast<pinocchio::FrameIndex>(frame_id)];
        Iso3 T = Iso3::Identity();
        T.linear() = M.rotation();
        T.translation() = M.translation();
        return T;
    }

    Iso3 ArmKinematics::tool_pose() const
    {
        return frame_pose(model_.tool_frame_id());
    }

    Iso3 ArmKinematics::ft_pose() const
    {
        return frame_pose(model_.ft_frame_id());
    }

    Iso3 ArmKinematics::tool_pose_in_mount() const
    {
        return frame_pose(model_.mount_frame_id()).inverse() * tool_pose();
    }

    ArmJacobian ArmKinematics::frame_jacobian(int frame_id) const
    {
        ArmJacobian J = ArmJacobian::Zero();
        pinocchio::getFrameJacobian(model_.model(), model_.data(), 
                                 static_cast<pinocchio::FrameIndex>(frame_id), 
                                       pinocchio::LOCAL_WORLD_ALIGNED, J);
        return J;
    }

    ArmJacobian ArmKinematics::tool_jacobian() const
    {
        return frame_jacobian(model_.tool_frame_id());
    }

    double ArmKinematics::manipulability() const
    {
        // TODO(M1)
        return 0.0;
    }

}  // namespace T1
