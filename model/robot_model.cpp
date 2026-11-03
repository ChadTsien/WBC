#include "model/robot_model.hpp"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace T1
{

    struct RobotModel::Impl
    {
        pinocchio::Model model;
        std::unique_ptr<pinocchio::Data> data;
        int tool_frame = -1;
        int ft_frame = -1;
    };

    RobotModel::RobotModel(const RobotModelConfig& /*config*/) : impl_(std::make_unique<Impl>())
    {
        // TODO(M0): build from URDF, lock wheel joints, check joint order, resolve frames
        impl_->data = std::make_unique<pinocchio::Data>(impl_->model);
    }

    RobotModel::~RobotModel() = default;

    const pinocchio::Model& RobotModel::model() const
    {
        return impl_->model;
    }

    pinocchio::Data& RobotModel::data()
    {
        return *impl_->data;
    }

    int RobotModel::tool_frame_id() const
    {
        return impl_->tool_frame;
    }

    int RobotModel::ft_frame_id() const
    {
        return impl_->ft_frame;
    }

    int RobotModel::frame_id(const std::string& /*name*/) const
    {
        // TODO(M0)
        return -1;
    }

    ArmVec RobotModel::lower_limits() const
    {
        // TODO(M0)
        return ArmVec::Zero();
    }

    ArmVec RobotModel::upper_limits() const
    {
        // TODO(M0)
        return ArmVec::Zero();
    }

    ArmVec RobotModel::velocity_limits() const
    {
        // TODO(M0)
        return ArmVec::Zero();
    }

}  // namespace T1
