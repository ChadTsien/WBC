#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "signal/brushes.hpp"
#include "signal/identify.hpp"
#include "wbc/msg/task_space_trajectory.hpp"
#include "wbc/msg/wbc_status.hpp"
#include "wbc/srv/calibrate_ft.hpp"
#include "wbc/srv/set_brush.hpp"

namespace T1::ft {

// Offline gravity calibration. Drives the brush through pitch poses via the WBC reference
// topic, averages raw F/T at each pose, solves GravityIdentifier, saves to BrushRegistry,
// then asks the WBC controller to reload (SetBrush). Not real-time.
class FtCalibrationNode : public rclcpp::Node {
 public:
  explicit FtCalibrationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void on_calibrate(const std::shared_ptr<::wbc::srv::CalibrateFt::Request> request,
                    std::shared_ptr<::wbc::srv::CalibrateFt::Response> response);

  // brush_tip targets in the odom frame, same position, pitch spread over the joint range.
  std::vector<Iso3> make_poses(std::size_t count) const;
  bool move_and_settle(const Iso3& target);
  std::optional<CalibrationSample> collect_sample();
  bool request_reload(const std::string& brush_id);

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr io_group_;
  rclcpp::Service<::wbc::srv::CalibrateFt>::SharedPtr calibrate_srv_;
  rclcpp::Client<::wbc::srv::SetBrush>::SharedPtr set_brush_client_;
  rclcpp::Publisher<::wbc::msg::TaskSpaceTrajectory>::SharedPtr reference_pub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr raw_wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<::wbc::msg::WbcStatus>::SharedPtr status_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex mutex_;
  std::vector<Wrench> wrench_window_;
  std::optional<sensor_msgs::msg::Imu> last_imu_;
  std::optional<::wbc::msg::WbcStatus> last_status_;

  std::unique_ptr<BrushRegistry> registry_;
};

}  // namespace T1::ft
