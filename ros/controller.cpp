#include "ros/controller.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace T1::wbc
{

    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using controller_interface::return_type;

    WbcRosController::CallbackReturn WbcRosController::on_init()
    {
        // TODO(M1): auto_declare arm_joints, wheel_joints, imu_name, ft_name, config_file,
        //           robot_description_node, odom_frame, base_frame, status_rate_hz
        return CallbackReturn::SUCCESS;
    }

    InterfaceConfiguration WbcRosController::command_interface_configuration() const
    {
        // TODO(M1): per arm joint position/velocity/effort/stiffness/damping, per wheel velocity
        return {interface_configuration_type::INDIVIDUAL, {}};
    }

    InterfaceConfiguration WbcRosController::state_interface_configuration() const
    {
        // TODO(M1): arm position/velocity/effort, wheel position/velocity, imu.*, ft_sensor.*
        return {interface_configuration_type::INDIVIDUAL, {}};
    }

    WbcRosController::CallbackReturn WbcRosController::on_configure(const rclcpp_lifecycle::State&)
    {
        // TODO(M1): load_wbc_config, fetch_robot_description, build model / wbc / estimator /
        //           ft chain, io_.configure
        return CallbackReturn::SUCCESS;
    }

    WbcRosController::CallbackReturn WbcRosController::on_activate(const rclcpp_lifecycle::State&)
    {
        // TODO(M1): reset estimator at origin, hold current pose, wbc_->reset
        return CallbackReturn::SUCCESS;
    }

    WbcRosController::CallbackReturn WbcRosController::on_deactivate(const rclcpp_lifecycle::State&)
    {
        // TODO(M1): write_hold
        return CallbackReturn::SUCCESS;
    }

    return_type WbcRosController::update(const rclcpp::Time&, const rclcpp::Duration&)
    {
        // TODO(M1): read_state -> current_reference -> wbc_->update -> write_command -> io_.publish
        return return_type::OK;
    }

    RobotState WbcRosController::read_state(double /*dt*/, Wrench* /*compensated_ft*/,
                                            ft::ContactResult* /*contact*/)
    {
        // TODO(M1)
        return RobotState{};
    }

    Reference WbcRosController::current_reference(double /*now_s*/)
    {
        // TODO(M1): sampled trajectory, teleop override of the base, else hold_reference_
        return hold_reference_;
    }

    void WbcRosController::write_command(const Output& /*out*/)
    {
        // TODO(M1)
    }

    void WbcRosController::write_hold()
    {
        // TODO(M1)
    }

}  // namespace T1::wbc

PLUGINLIB_EXPORT_CLASS(T1::wbc::WbcRosController, controller_interface::ControllerInterface)
