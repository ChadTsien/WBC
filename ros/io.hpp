#pragma once

#include <atomic>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>

#include "control/types.hpp"
#include "signal/brushes.hpp"
#include "signal/contact.hpp"
#include "wbc/msg/contact_state.hpp"
#include "wbc/msg/ft_status.hpp"
#include "wbc/msg/task_space_trajectory.hpp"
#include "wbc/msg/wbc_status.hpp"
#include "wbc/srv/set_brush.hpp"
#include "wbc/srv/tare_ft.hpp"

namespace T1::wbc
{

    struct StampedTrajectory
    {
        double start_time = 0.0;  // s, ROS time
        Trajectory points;
    };

    struct IoConfig
    {
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
        double status_rate_hz = 100.0;
        double teleop_timeout_s = 0.5;
        std::string brushes_yaml;
    };

    // Everything ROS-facing of the WBC controller. Callbacks run in non-RT threads and hand
    // data to the RT update() through realtime buffers / atomics; publishing from update()
    // goes through RealtimePublisher (try-lock, never blocks).
    class WbcRosIo
    {
        public:
        void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                       const IoConfig& config);

        // --- RT side, called from update() ---
        std::shared_ptr<const StampedTrajectory> trajectory();
        std::optional<BaseTwist> teleop(double now_s);
        bool take_tare_request();
        std::optional<ft::ToolGravityParams> take_brush_change(std::string* brush_id);

        void publish(double now_s, const RobotState& state, const Output& out,
                     const ft::ContactResult& contact, const Wrench& compensated_ft);

        private:
        void on_reference(const ::wbc::msg::TaskSpaceTrajectory::SharedPtr msg);
        void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
        void on_tare(const std::shared_ptr<::wbc::srv::TareFt::Request> request,
                     std::shared_ptr<::wbc::srv::TareFt::Response> response);
        void on_set_brush(const std::shared_ptr<::wbc::srv::SetBrush::Request> request,
                          std::shared_ptr<::wbc::srv::SetBrush::Response> response);

        IoConfig config_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        std::unique_ptr<ft::BrushRegistry> registry_;

        rclcpp::Subscription<::wbc::msg::TaskSpaceTrajectory>::SharedPtr reference_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Service<::wbc::srv::TareFt>::SharedPtr tare_srv_;
        rclcpp::Service<::wbc::srv::SetBrush>::SharedPtr set_brush_srv_;

        realtime_tools::RealtimeBuffer<std::shared_ptr<const StampedTrajectory>> trajectory_buffer_;
        struct TimedTwist
        {
            BaseTwist twist;
            double stamp_s = -1.0;
        };
        realtime_tools::RealtimeBuffer<TimedTwist> teleop_buffer_;
        struct BrushChange
        {
            std::string id;
            ft::ToolGravityParams params;
            std::uint64_t seq = 0;
        };
        realtime_tools::RealtimeBuffer<BrushChange> brush_buffer_;
        std::uint64_t brush_seq_applied_ = 0;
        std::atomic<bool> tare_requested_{false};

        using StatusPub = realtime_tools::RealtimePublisher<::wbc::msg::WbcStatus>;
        using ContactPub = realtime_tools::RealtimePublisher<::wbc::msg::ContactState>;
        using FtStatusPub = realtime_tools::RealtimePublisher<::wbc::msg::FtStatus>;
        using WrenchPub = realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>;
        using OdomPub = realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>;
        using TfPub = realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>;
        std::unique_ptr<StatusPub> status_pub_;
        std::unique_ptr<ContactPub> contact_pub_;
        std::unique_ptr<FtStatusPub> ft_status_pub_;
        std::unique_ptr<WrenchPub> wrench_pub_;
        std::unique_ptr<OdomPub> odom_pub_;
        std::unique_ptr<TfPub> tf_pub_;
        double last_status_s_ = 0.0;
    };

    // Reads the URDF from another node's "robot_description" parameter (robot_state_publisher).
    // Spins a temporary node; call from on_configure only. Returns empty string on timeout.
    std::string fetch_robot_description(const std::string& node_name, std::chrono::seconds timeout);

}  // namespace T1::wbc
