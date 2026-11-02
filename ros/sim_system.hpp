#pragma once

#include <memory>
#include <vector>

#include <hardware_interface/system_interface.hpp>

#include "sim/world.hpp"
#include "sim/sensors.hpp"
#include "sim/viewer.hpp"

namespace T1::sim {

// ros2_control plugin T1::sim::SimSystem. Same interfaces as T1::hal::HardwareSystem
// (see config/urdf/ros2_control.xacro).
// read() advances physics by one controller period (several MuJoCo substeps).
class SimSystem : public hardware_interface::SystemInterface {
 public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

 private:
  std::unique_ptr<MujocoWorld> world_;
  std::unique_ptr<SimViewer> viewer_;
  std::unique_ptr<FtSensorModel> ft_model_;
  std::unique_ptr<ImuModel> imu_model_;

  // Interface storage, index order = joint order in HardwareInfo.
  std::vector<double> arm_cmd_position_, arm_cmd_velocity_, arm_cmd_effort_;
  std::vector<double> arm_cmd_stiffness_, arm_cmd_damping_;
  std::vector<double> arm_state_position_, arm_state_velocity_, arm_state_effort_;
  std::vector<double> wheel_cmd_velocity_, wheel_state_position_, wheel_state_velocity_;
  std::vector<double> imu_state_;  // orientation xyzw, angular_velocity xyz, linear_acceleration xyz
  std::vector<double> ft_state_;   // force xyz, torque xyz
};

}  // namespace T1::sim
