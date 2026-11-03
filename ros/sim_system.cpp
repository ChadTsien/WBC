#include "ros/sim_system.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace T1::sim
{

    using hardware_interface::CommandInterface;
    using hardware_interface::return_type;
    using hardware_interface::StateInterface;

    SimSystem::CallbackReturn SimSystem::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
        // TODO(M0): validate joints / sensors against ros2_control.xacro, size storage,
        //           read params (mjcf_path, noise), create world and sensor models
        return CallbackReturn::SUCCESS;
    }

    SimSystem::CallbackReturn SimSystem::on_activate(const rclcpp_lifecycle::State&)
    {
        // TODO(M0): hold current pose: command position = state position, start viewer
        return CallbackReturn::SUCCESS;
    }

    SimSystem::CallbackReturn SimSystem::on_deactivate(const rclcpp_lifecycle::State&)
    {
        // TODO(M0): stop viewer
        return CallbackReturn::SUCCESS;
    }

    std::vector<StateInterface> SimSystem::export_state_interfaces()
    {
        // TODO(M0)
        return {};
    }

    std::vector<CommandInterface> SimSystem::export_command_interfaces()
    {
        // TODO(M0)
        return {};
    }

    return_type SimSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
    {
        // TODO(M0): step physics for one period, copy state into interface storage
        return return_type::OK;
    }

    return_type SimSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
    {
        // TODO(M0): copy command storage into world_->set_arm_command / set_wheel_command
        return return_type::OK;
    }

}  // namespace T1::sim

PLUGINLIB_EXPORT_CLASS(T1::sim::SimSystem, hardware_interface::SystemInterface)
