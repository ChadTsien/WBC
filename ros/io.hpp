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
    /**
     * 带 ROS 起始时间的轨迹
     *
     * 成员
     * - start_time：轨迹 header.stamp，s
     * - points：按 t 升序，给 sample_reference
     */
    struct StampedTrajectory
    {
        double start_time = 0.0;
        Trajectory points;
    };

    /**
     * WbcRosIo 参数
     *
     * 成员
     * - odom_frame / base_frame：发布 tf 用
     * - status_rate_hz：status 降频
     * - teleop_timeout_s：超过则视为遥控断开
     * - brushes_yaml：M3a 工具参数文件
     */
    struct IoConfig
    {
        std::string odom_frame = "odom";
        std::string base_frame = "base_link";
        double status_rate_hz = 100.0;
        double teleop_timeout_s = 0.5;
        std::string brushes_yaml;
    };

    /**
     * 控制器所有面向 ROS 的部分
     *
     * 说明
     * - 回调在非实时线程；与 update() 之间只用 RealtimeBuffer / atomic
     * - 发布用 RealtimePublisher：trylock 成功才写，永不阻塞实时线程
     * - tare / set_brush 的文件 I/O 在服务线程，M3a 再接满
     */
    class WbcRosIo
    {
        public:
            /**
             * 建订阅、服务、实时发布者；非实时，on_configure 里调用
             */
            void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                           const IoConfig& config);

            /**
             * 实时侧：当前轨迹；还没有则为空指针
             */
            std::shared_ptr<const StampedTrajectory> trajectory();

            /**
             * 实时侧：未超时的遥控 [v, ω]
             *
             * 参数
             * - now_s：当前 ROS 时间，s
             *
             * 返回：超时或从未收到则 nullopt
             */
            std::optional<ChassisTwist> teleop(double now_s);

            /**
             * 实时侧：取出标零请求（exchange false）
             */
            bool take_tare_request();

            /**
             * 实时侧：若序号更新则取出新工具参数，只处理一次
             *
             * 参数
             * - brush_id：非空时写入工具名
             */
            std::optional<ft::ToolGravityParams> take_brush_change(std::string* brush_id);

            /**
             * 实时侧发布
             *
             * 说明
             * - 每拍 odom 和 odom→base_link 的 tf
             * - status / contact / ft / wrench 按 status_rate_hz 降频
             */
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
                ChassisTwist twist;
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

    /**
     * 从其它节点的 robot_description 参数读 URDF
     *
     * 说明
     * - 给 robot_state_publisher；临时节点 spin，只在 on_configure 调用
     * - 超时返回空字符串
     */
    std::string fetch_robot_description(const std::string& node_name, std::chrono::seconds timeout);

}  // namespace T1::wbc
