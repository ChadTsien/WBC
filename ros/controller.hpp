#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>

#include "estimation/odometry.hpp"
#include "estimation/ekf.hpp"
#include "signal/contact.hpp"
#include "signal/compensate.hpp"
#include "model/robot_model.hpp"
#include "control/controller.hpp"
#include "ros/io.hpp"

namespace T1::wbc {

// ros2_control plugin T1::wbc::WbcRosController. The single 500 Hz in-loop owner of:
// state estimation, F/T compensation, contact detection and the whole-body QP.
// Interfaces are loaned in the order of *_interface_configuration(); the index helpers in
// the .cpp rely on that order.
class WbcRosController : public controller_interface::ControllerInterface {
 public:
  CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  // State interfaces -> estimator -> F/T chain -> RobotState.
  RobotState read_state(double dt, Wrench* compensated_ft, ft::ContactResult* contact);
  Reference current_reference(double now_s);
  void write_command(const Output& out);
  // Activation / failure fallback: hold position with free-motion gains, base stopped.
  void write_hold();

  std::vector<std::string> arm_joints_;
  std::vector<std::string> wheel_joints_;
  std::string imu_name_;
  std::string ft_name_;

  WbcConfig config_;
  std::unique_ptr<RobotModel> model_;
  std::unique_ptr<WbcController> wbc_;
  std::unique_ptr<est::DiffDriveOdometry> odometry_;
  est::PlanarEkf ekf_;
  ft::FtCompensator ft_compensator_;
  ft::ContactDetector contact_detector_;
  WbcRosIo io_;

  Reference hold_reference_;
};

}  // namespace T1::wbc
