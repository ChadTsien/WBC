#include "ros/io.hpp"

namespace T1::wbc
{

    void WbcRosIo::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                             const IoConfig& config)
    {
        node_ = node;
        config_ = config;
        // TODO(M1): subscriptions, services, realtime publishers (~/status, ~/contact,
        //           ~/ft_status, ~/wrench_compensated, /odom, /tf), registry_->load()
    }

    std::shared_ptr<const StampedTrajectory> WbcRosIo::trajectory()
    {
        return *trajectory_buffer_.readFromRT();
    }

    std::optional<BaseTwist> WbcRosIo::teleop(double /*now_s*/)
    {
        // TODO(M1)
        return std::nullopt;
    }

    bool WbcRosIo::take_tare_request()
    {
        return tare_requested_.exchange(false);
    }

    std::optional<ft::ToolGravityParams> WbcRosIo::take_brush_change(std::string* /*brush_id*/)
    {
        // TODO(M3a)
        return std::nullopt;
    }

    void WbcRosIo::publish(double /*now_s*/, const RobotState& /*state*/, const Output& /*out*/,
                           const ft::ContactResult& /*contact*/, const Wrench& /*compensated_ft*/)
    {
        // TODO(M1): odom + tf every cycle, the rest decimated to status_rate_hz
    }

    void WbcRosIo::on_reference(const ::wbc::msg::TaskSpaceTrajectory::SharedPtr /*msg*/)
    {
        // TODO(M1): convert to StampedTrajectory (allocation is fine here), writeFromNonRT
    }

    void WbcRosIo::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr /*msg*/)
    {
        // TODO(M1)
    }

    void WbcRosIo::on_tare(const std::shared_ptr<::wbc::srv::TareFt::Request> /*request*/,
                           std::shared_ptr<::wbc::srv::TareFt::Response> response)
    {
        // TODO(M3a)
        response->success = false;
        response->message = "not implemented";
    }

    void WbcRosIo::on_set_brush(const std::shared_ptr<::wbc::srv::SetBrush::Request> /*request*/,
                                std::shared_ptr<::wbc::srv::SetBrush::Response> response)
    {
        // TODO(M3a)
        response->success = false;
        response->message = "not implemented";
    }

    std::string fetch_robot_description(const std::string& /*node_name*/,
                                        std::chrono::seconds /*timeout*/)
    {
        // TODO(M1)
        return {};
    }

}  // namespace T1::wbc
