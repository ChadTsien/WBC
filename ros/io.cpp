#include "ros/io.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>

#include "control/types.hpp"
#include "types/types.hpp"
#include "wbc/msg/detail/task_space_point__struct.hpp"

#include <builtin_interfaces/msg/detail/duration__struct.hpp>
#include <builtin_interfaces/msg/detail/time__struct.hpp>
#include <cstdint>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>

namespace T1::wbc
{

    namespace  
    {
        double duration_s(const builtin_interfaces::msg::Duration& d)
        {
            return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
        }

        Iso3 pose_to_iso(const geometry_msgs::msg::Pose& p)
        {
            Iso3 T = Iso3::Identity();
            T.translation() = Vec3(p.position.x, p.position.y, p.position.z);
            T.linear() = Quat(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z)
                             .normalized()
                             .toRotationMatrix();
            return T;
        }

        Reference from_msg(const ::wbc::msg::TaskSpacePoint& p)
        {
            Reference r;
            r.chassis_mode = static_cast<ChassisMode>(p.chassis_mode);
            r.chassis_pose.x = p.chassis_x;
            r.chassis_pose.y = p.chassis_y;
            r.chassis_pose.yaw = p.chassis_yaw;
            r.chassis_twist.v = p.chassis_v;
            r.chassis_twist.omega = p.chassis_omega;
            r.ee_pose = pose_to_iso(p.ee_pose);
            r.ee_twist.linear = Vec3(p.ee_twist.linear.x, p.ee_twist.linear.y, p.ee_twist.linear.z);
            r.ee_twist.angular =
                Vec3(p.ee_twist.angular.x, p.ee_twist.angular.y, p.ee_twist.angular.z);
            r.contact_phase = static_cast<ContactPhase>(p.contact_phase);
            r.surface_normal = Vec3(p.surface_normal.x, p.surface_normal.y, p.surface_normal.z);
            r.desired_normal_force = p.desired_normal_force;
            return r;
        }

        builtin_interfaces::msg::Time to_stamp(double t)
        {
            builtin_interfaces::msg::Time stamp;
            if (t < 0.0)
            {
                return stamp;
            }
            stamp.sec = static_cast<std::int32_t>(t);
            stamp.nanosec = 
                static_cast<std::uint32_t>((t - static_cast<double>(stamp.sec)) * 1e9);
            return stamp;
        }

        void fill_header(std_msgs::msg::Header& header, double now_s, const std::string& frame)
        {
            header.stamp = to_stamp(now_s);
            header.frame_id = frame;
        }

        void fill_yaw(geometry_msgs::msg::Quaternion& q, double yaw)
        {
            const Quat r(Eigen::AngleAxisd(yaw, Vec3::UnitZ()));
            q.x = r.x();
            q.y = r.y();
            q.z = r.z();
            q.w = r.w();
        }

        void fill_wrench(geometry_msgs::msg::Wrench& dst, const Wrench& src)
        {
            dst.force.x = src.force.x();
            dst.force.y = src.force.y();
            dst.force.z = src.force.z();
            dst.torque.x = src.torque.x();
            dst.torque.y = src.torque.y();
            dst.torque.z = src.torque.z();
        }
    }

    void WbcRosIo::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                             const IoConfig& config)
    {
        node_ = node;
        config_ = config;
        if (!config_.brushes_yaml.empty())
        {
            registry_ = std::make_unique<ft::BrushRegistry>(config_.brushes_yaml);
            std::string err;
            if (!registry_->load(&err))
            {
                RCLCPP_WARN(node_->get_logger(), "brush registry: %s", err.c_str());
            }
        }
        reference_sub_ = node_->create_subscription<::wbc::msg::TaskSpaceTrajectory>(
            "~/reference", 10,
            std::bind(&WbcRosIo::on_reference, this, std::placeholders::_1));
        cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "~/cmd_vel", 10, std::bind(&WbcRosIo::on_cmd_vel, this, std::placeholders::_1));
        
            tare_srv_ = node_->create_service<::wbc::srv::TareFt>(
            "~/tare_ft",
            std::bind(&WbcRosIo::on_tare, this, std::placeholders::_1, std::placeholders::_2));
        set_brush_srv_ = node_->create_service<::wbc::srv::SetBrush>(
            "~/set_brush", std::bind(&WbcRosIo::on_set_brush, this, std::placeholders::_1,
                                     std::placeholders::_2));
        
        status_pub_ = std::make_unique<StatusPub>(
            node_->create_publisher<::wbc::msg::WbcStatus>("~/status", 10));
        contact_pub_ = std::make_unique<ContactPub>(
            node_->create_publisher<::wbc::msg::ContactState>("~/contact", 10));
        ft_status_pub_ = std::make_unique<FtStatusPub>(
            node_->create_publisher<::wbc::msg::FtStatus>("~/ft_status", 10));
        wrench_pub_ = std::make_unique<WrenchPub>(
            node_->create_publisher<geometry_msgs::msg::WrenchStamped>("~/wrench_compensated", 10));
        odom_pub_ = std::make_unique<OdomPub>(
            node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10));
        tf_pub_ = std::make_unique<TfPub>(
            node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100));
    }

    std::shared_ptr<const StampedTrajectory> WbcRosIo::trajectory()
    {
        return *trajectory_buffer_.readFromRT();
    }

    std::optional<ChassisTwist> WbcRosIo::teleop(double now_s)
    {
        const TimedTwist t = *teleop_buffer_.readFromRT();
        if (t.stamp_s < 0.0)
        {
            return std::nullopt;
        }
        if (now_s - t.stamp_s > config_.teleop_timeout_s)
        {
            return std::nullopt;
        }
        return t.twist;
    }

    bool WbcRosIo::take_tare_request()
    {
        return tare_requested_.exchange(false);
    }

    std::optional<ft::ToolGravityParams> WbcRosIo::take_brush_change(std::string* brush_id)
    {
        const BrushChange ch = *brush_buffer_.readFromRT();
        if (ch.seq == brush_seq_applied_)
        {
            return std::nullopt;
        }
        brush_seq_applied_ = ch.seq;
        if (brush_id)
        {
            *brush_id = ch.id;
        }
        return ch.params;
    }

    void WbcRosIo::publish(double now_s, const RobotState& state, const Output& out,
                           const ft::ContactResult& contact, const Wrench& compensated_ft)
    {
        const auto stamp = to_stamp(now_s);
        if (odom_pub_ && odom_pub_->trylock())
        {
            auto& msg = odom_pub_->msg_;
            fill_header(msg.header, now_s, config_.odom_frame);
            msg.child_frame_id = config_.base_frame;
            msg.pose.pose.position.x = state.chassis_pose.x;
            msg.pose.pose.position.y = state.chassis_pose.y;
            msg.pose.pose.position.z = 0.0;
            fill_yaw(msg.pose.pose.orientation, state.chassis_pose.yaw);
            msg.twist.twist.linear.x = state.chassis_twist.v;
            msg.twist.twist.linear.y = 0.0;
            msg.twist.twist.linear.z = 0.0;
            msg.twist.twist.angular.x = 0.0;
            msg.twist.twist.angular.y = 0.0;
            msg.twist.twist.angular.z = state.chassis_twist.omega;
            odom_pub_->unlockAndPublish();
        }

        if (tf_pub_ && tf_pub_->trylock())
        {
            auto& msg = tf_pub_->msg_;
            msg.transforms.resize(1);
            auto& tf = msg.transforms.front();
            tf.header.stamp = stamp;
            tf.header.frame_id = config_.odom_frame;
            tf.child_frame_id = config_.base_frame;
            tf.transform.translation.x = state.chassis_pose.x;
            tf.transform.translation.y = state.chassis_pose.y;
            tf.transform.translation.z = 0.0;
            fill_yaw(tf.transform.rotation, state.chassis_pose.yaw);
            tf_pub_->unlockAndPublish();
        }

        const double period = 1.0 / std::max(config_.status_rate_hz, 1.0);
        if (now_s - last_status_s_ < period)
        {
            return;
        }
        last_status_s_ = now_s;

        if (status_pub_ && status_pub_->trylock())
        {
            auto& msg = status_pub_->msg_;
            fill_header(msg.header, now_s, config_.odom_frame);
            msg.qp_status = static_cast<std::uint8_t>(out.qp_status);
            msg.qp_iterations = static_cast<std::uint32_t>(out.qp_iterations);
            msg.qp_solve_time_us = out.qp_solve_time_us;
            msg.cycle_time_us = 0.0;
            for (int i = 0; i < kTaskDim; ++i)
            {
                msg.ee_error[static_cast<std::size_t>(i)] = out.ee_error[i];
            }
            msg.chassis_v_cmd = out.chassis.v;
            msg.chassis_omega_cmd = out.chassis.omega;
            msg.joint_velocity_cmd.resize(static_cast<std::size_t>(kNumArmJoints));
            for (int i = 0; i < kNumArmJoints; ++i)
            {
                msg.joint_velocity_cmd[static_cast<std::size_t>(i)] =
                    out.qp_solution[kNumChassisInputs + i];
            }
            msg.admittance_offset.x = out.admittance_offset.x();
            msg.admittance_offset.y = out.admittance_offset.y();
            msg.admittance_offset.z = out.admittance_offset.z();
            msg.manipulability = out.manipulability;
            msg.active_constraints.clear();
            status_pub_->unlockAndPublish();
        }

        if (contact_pub_ && contact_pub_->trylock())
        {
            auto& msg = contact_pub_->msg_;
            fill_header(msg.header, now_s, config_.odom_frame);
            msg.phase = 0;
            msg.in_contact = contact.in_contact;
            msg.brush_detached = contact.brush_detached;
            msg.normal_force = contact.normal_force;
            fill_wrench(msg.wrench_world, compensated_ft);
            contact_pub_->unlockAndPublish();
        }

        if (ft_status_pub_ && ft_status_pub_->trylock())
        {
            auto& msg = ft_status_pub_->msg_;
            fill_header(msg.header, now_s, config_.base_frame);
            ft_status_pub_->unlockAndPublish();
        }

        if (wrench_pub_ && wrench_pub_->trylock())
        {
            auto& msg = wrench_pub_->msg_;
            fill_header(msg.header, now_s, config_.odom_frame);
            fill_wrench(msg.wrench, compensated_ft);
            wrench_pub_->unlockAndPublish();
        }
    }

    void WbcRosIo::on_reference(const ::wbc::msg::TaskSpaceTrajectory::SharedPtr msg)
    {
        auto traj = std::make_shared<StampedTrajectory>();
        traj->start_time = static_cast<double>(msg->header.stamp.sec)
                           + 1e-9 * static_cast<double>(msg->header.stamp.nanosec);
        traj->points.reserve(msg->points.size());
        for (const auto& p : msg->points)
        {
            TimedReference tr;
            tr.t = duration_s(p.time_from_start);
            tr.ref = from_msg(p);
            traj->points.push_back(tr);
        }
        trajectory_buffer_.writeFromNonRT(
            std::shared_ptr<const StampedTrajectory>(std::move(traj)));
    }

    void WbcRosIo::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!node_)
        {
            return;
        }
        TimedTwist t;
        t.twist.v = msg->linear.x;
        t.twist.omega = msg->angular.z;
        t.stamp_s = node_->now().seconds();
        teleop_buffer_.writeFromNonRT(t);
    }

    void WbcRosIo::on_tare(const std::shared_ptr<::wbc::srv::TareFt::Request> /*request*/,
        std::shared_ptr<::wbc::srv::TareFt::Response> response)
    {
        tare_requested_.store(true);
        response->success = true;
        response->message = "queued";
    }

    void WbcRosIo::on_set_brush(const std::shared_ptr<::wbc::srv::SetBrush::Request> request,
                                std::shared_ptr<::wbc::srv::SetBrush::Response> response)
    {
        if (!registry_)
        {
            response->success = false;
            response->message = "no registry";
            return;
        }
        const auto params = registry_->get(request->brush_id);
        if (!params)
        {
            response->success = false;
            response->message = "unknown brush";
            return;
        }
        BrushChange ch;
        ch.id = request->brush_id;
        ch.params = *params;
        ch.seq = brush_buffer_.readFromNonRT()->seq + 1;
        brush_buffer_.writeFromNonRT(ch);
        response->success = true;
        response->message = "ok";
    }

    std::string fetch_robot_description(const std::string& node_name,
                                        std::chrono::seconds timeout)
    {
        auto node = std::make_shared<rclcpp::Node>("wbc_urdf_client");
        auto client = node->create_client<rcl_interfaces::srv::GetParameters>(
            node_name + "/get_parameters");
        if (!client->wait_for_service(timeout))
        {
            return {};
        }
        auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
        request->names = {"robot_description"};
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future, timeout)
            != rclcpp::FutureReturnCode::SUCCESS)
        {
            return {};
        }
        const auto response = future.get();
        if (response->values.empty()
            || response->values.front().type != rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
        {
            return {};
        }
        return response->values.front().string_value;
    }

}  // namespace T1::wbc
