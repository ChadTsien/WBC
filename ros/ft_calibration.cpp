#include "ros/ft_calibration.hpp"

namespace T1::ft {

FtCalibrationNode::FtCalibrationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("ft_calibration", options) {
  // TODO(M3a): parameters (brushes_yaml, topics, settle tolerances), callback groups,
  //            service / client / pub / subs, TF listener, registry_->load()
}

void FtCalibrationNode::on_calibrate(
    const std::shared_ptr<::wbc::srv::CalibrateFt::Request> /*request*/,
    std::shared_ptr<::wbc::srv::CalibrateFt::Response> response) {
  // TODO(M3a)
  response->success = false;
  response->message = "not implemented";
}

std::vector<Iso3> FtCalibrationNode::make_poses(std::size_t /*count*/) const {
  // TODO(M3a)
  return {};
}

bool FtCalibrationNode::move_and_settle(const Iso3& /*target*/) {
  // TODO(M3a)
  return false;
}

std::optional<CalibrationSample> FtCalibrationNode::collect_sample() {
  // TODO(M3a)
  return std::nullopt;
}

bool FtCalibrationNode::request_reload(const std::string& /*brush_id*/) {
  // TODO(M3a)
  return false;
}

}  // namespace T1::ft
