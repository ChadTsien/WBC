#include "ros/controller.hpp"

#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "control/reference.hpp"
#include "control/types.hpp"
#include "estimation/tilt.hpp"
#include "kine/arm.hpp"
#include "model/chassis_model.hpp"
#include "types/types.hpp"

namespace T1::wbc
{

    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using controller_interface::return_type;

    namespace
    {
        const char* kArmCmd[] = {"position", "velocity", "effort", "stiffness", "damping"};
        const char* kArmState[] = {"position", "velocity", "effort"};
        const char* kWheelState[] = {"position", "velocity"};
        const char* kImuState[] = {
            "orientation.x", "orientation.y", "orientation.z", "orientation.w",
            "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
            "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
        const char* kFtState[] = {
            "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};

        constexpr int kArmCmdN = 5;
        constexpr int kArmStateN = 3;
        constexpr int kWheelN = 2;
        constexpr int kWheelStateN = 2;
        constexpr int kImuN = 10;
        constexpr int kFtN = 6;
        constexpr int kCmdCount = kNumArmJoints * kArmCmdN + kWheelN;
        constexpr int kStateCount =
            kNumArmJoints * kArmStateN + kWheelN * kWheelStateN + kImuN + kFtN;

        int cmd_arm(int joint, int field)
        {
            return joint * kArmCmdN + field;
        }

        int cmd_wheel(int wheel)
        {
            return kNumArmJoints * kArmCmdN + wheel;
        }

        int st_arm(int joint, int field)
        {
            return joint * kArmStateN + field;
        }

        int st_wheel(int wheel, int field)
        {
            return kNumArmJoints * kArmStateN + wheel * kWheelStateN + field;
        }

        int st_imu(int field)
        {
            return kNumArmJoints * kArmStateN + kWheelN * kWheelStateN + field;
        }

        int st_ft(int field)
        {
            return st_imu(0) + kImuN + field;
        }

        Iso3 T_odom_chassis(const Pose2d& pose)
        {
            Iso3 T = Iso3::Identity();
            T.translation() = Vec3(pose.x, pose.y, 0.0);
            T.linear() = Eigen::AngleAxisd(pose.yaw, Vec3::UnitZ()).toRotationMatrix();
            return T;
        }
    }  // namespace

    WbcRosController::CallbackReturn WbcRosController::on_init()
    {
        try
        {
            arm_joints_ = auto_declare<std::vector<std::string>>(
                "arm_joints", {"arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4",
                               "arm_joint5", "arm_joint6"});
            wheel_joints_ = auto_declare<std::vector<std::string>>(
                "wheel_joints", {"wheel_left_joint", "wheel_right_joint"});
            imu_name_ = auto_declare<std::string>("imu_name", "imu");
            ft_name_ = auto_declare<std::string>("ft_name", "ft_sensor");
            auto_declare<std::string>("config_file", "");
            auto_declare<std::string>("robot_description_node", "robot_state_publisher");
            auto_declare<std::string>("odom_frame", "odom");
            auto_declare<std::string>("base_frame", "base_link");
            auto_declare<double>("status_rate_hz", 100.0);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "on_init: %s", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    InterfaceConfiguration WbcRosController::command_interface_configuration() const
    {
        InterfaceConfiguration conf;
        conf.type = interface_configuration_type::INDIVIDUAL;
        for (const auto& joint : arm_joints_)
        {
            for (const char* iface : kArmCmd)
            {
                conf.names.push_back(joint + "/" + iface);
            }
        }
        for (const auto& joint : wheel_joints_)
        {
            conf.names.push_back(joint + "/velocity");
        }
        return conf;
    }

    InterfaceConfiguration WbcRosController::state_interface_configuration() const
    {
        InterfaceConfiguration conf;
        conf.type = interface_configuration_type::INDIVIDUAL;
        for (const auto& joint : arm_joints_)
        {
            for (const char* iface : kArmState)
            {
                conf.names.push_back(joint + "/" + iface);
            }
        }
        for (const auto& joint : wheel_joints_)
        {
            for (const char* iface : kWheelState)
            {
                conf.names.push_back(joint + "/" + iface);
            }
        }
        for (const char* iface : kImuState)
        {
            conf.names.push_back(imu_name_ + "/" + iface);
        }
        for (const char* iface : kFtState)
        {
            conf.names.push_back(ft_name_ + "/" + iface);
        }
        return conf;
    }

    WbcRosController::CallbackReturn WbcRosController::on_configure(
        const rclcpp_lifecycle::State&)
    {
        if (arm_joints_.size() != static_cast<std::size_t>(kNumArmJoints)
            || wheel_joints_.size() != static_cast<std::size_t>(kWheelN))
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "expected %d arm joints and %d wheels", kNumArmJoints, kWheelN);
            return CallbackReturn::ERROR;
        }

        try
        {
            std::string yaml_path = get_node()->get_parameter("config_file").as_string();
            if (yaml_path.empty())
            {
                yaml_path = ament_index_cpp::get_package_share_directory("wbc") + "/wbc.yaml";
            }
            config_ = load_wbc_config(yaml_path);

            const std::string desc_node =
                get_node()->get_parameter("robot_description_node").as_string();
            const std::string urdf =
                fetch_robot_description(desc_node, std::chrono::seconds(5));
            if (urdf.empty())
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "empty robot_description from '%s'", desc_node.c_str());
                return CallbackReturn::ERROR;
            }

            ArmModelConfig model_config;
            model_config.urdf_xml = urdf;
            model_config.arm_joints = arm_joints_;
            model_ = std::make_unique<ArmModel>(model_config);
            wbc_ = std::make_unique<WbcController>(config_, *model_);
            odometry_ = std::make_unique<est::DiffDriveOdometry>(ChassisModel(config_.chassis));
            ekf_.configure(config_.ekf);
            ft_compensator_.configure(config_.ft);
            contact_detector_.configure(config_.contact_detector);

            IoConfig io;
            io.odom_frame = get_node()->get_parameter("odom_frame").as_string();
            io.base_frame = get_node()->get_parameter("base_frame").as_string();
            io.status_rate_hz = get_node()->get_parameter("status_rate_hz").as_double();
            io.brushes_yaml = config_.brushes_yaml;
            io_.configure(get_node(), io);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "on_configure: %s", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    WbcRosController::CallbackReturn WbcRosController::on_activate(
        const rclcpp_lifecycle::State&)
    {
        if (!wbc_ || !model_ || !odometry_)
        {
            return CallbackReturn::ERROR;
        }
        if (state_interfaces_.size() < static_cast<std::size_t>(kStateCount))
        {
            RCLCPP_ERROR(get_node()->get_logger(), "missing state interfaces");
            return CallbackReturn::ERROR;
        }

        RobotState state;
        for (int i = 0; i < kNumArmJoints; ++i)
        {
            state.arm.q[i] = state_interfaces_[st_arm(i, 0)].get_value();
            state.arm.dq[i] = state_interfaces_[st_arm(i, 1)].get_value();
            state.arm.tau[i] = state_interfaces_[st_arm(i, 2)].get_value();
        }
        WheelSpeeds wheel_pos;
        wheel_pos.left = state_interfaces_[st_wheel(0, 0)].get_value();
        wheel_pos.right = state_interfaces_[st_wheel(1, 0)].get_value();
        odometry_->reset(Pose2d{}, wheel_pos);
        ekf_.reset(Pose2d{});
        contact_detector_.reset();

        state.chassis_pose = ekf_.pose();
        state.chassis_twist = ekf_.twist();
        wbc_->reset(state);

        hold_reference_ = Reference{};
        mode_ = ControlMode::kHold;
        have_hold_ = false;
        snapshot_hold(state);

        write_hold();
        return CallbackReturn::SUCCESS;
    }

    WbcRosController::CallbackReturn WbcRosController::on_deactivate(
        const rclcpp_lifecycle::State&)
    {
        write_hold();
        return CallbackReturn::SUCCESS;
    }

    return_type WbcRosController::update(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        if (!wbc_ || !model_ || !odometry_)
        {
            return return_type::ERROR;
        }

        double dt = period.seconds();
        if (dt <= 0.0)
        {
            dt = 1.0 / std::max(config_.control_rate_hz, 1.0);
        }
        const double now_s = time.seconds();

        std::string brush_id;
        if (const auto params = io_.take_brush_change(&brush_id))
        {
            wbc_->set_tool_payload(params->mass, params->com);
            ft_compensator_.set_params(*params);
        }
        if (io_.take_tare_request())
        {
            ft_compensator_.request_tare();
        }

        Wrench compensated_ft;
        ft::ContactResult contact;
        const RobotState state = read_state(dt, &compensated_ft, &contact);
        const Reference ref = current_reference(now_s, state);

        if (contact.brush_detached)
        {
            write_hold();
            io_.publish(now_s, state, Output{}, contact, compensated_ft);
            return return_type::OK;
        }

        Output out = wbc_->update(state, ref, dt);
        if (out.qp_status != QpStatus::kOk)
        {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(), *get_node()->get_clock(), 1000,
                "QP status=%d iter=%d us=%.1f dt=%.6f q=[%.3f %.3f %.3f %.3f %.3f %.3f] "
                "finite_q=%d",
                static_cast<int>(out.qp_status), out.qp_iterations, out.qp_solve_time_us, dt,
                state.arm.q[0], state.arm.q[1], state.arm.q[2], state.arm.q[3],
                state.arm.q[4], state.arm.q[5], static_cast<int>(state.arm.q.allFinite()));
        }
        if (state.arm.q.allFinite())
        {
            write_command(out);
        }
        else
        {
            write_hold();
        }
        io_.publish(now_s, state, out, contact, compensated_ft);
        return return_type::OK;
    }

    RobotState WbcRosController::read_state(double dt, Wrench* compensated_ft,
                                            ft::ContactResult* contact)
    {
        RobotState state;
        for (int i = 0; i < kNumArmJoints; ++i)
        {
            state.arm.q[i] = state_interfaces_[st_arm(i, 0)].get_value();
            state.arm.dq[i] = state_interfaces_[st_arm(i, 1)].get_value();
            state.arm.tau[i] = state_interfaces_[st_arm(i, 2)].get_value();
        }

        WheelSpeeds wheel_pos;
        wheel_pos.left = state_interfaces_[st_wheel(0, 0)].get_value();
        wheel_pos.right = state_interfaces_[st_wheel(1, 0)].get_value();
        odometry_->update(wheel_pos, dt);
        ekf_.predict(dt);
        ekf_.update_odometry(odometry_->twist());

        Quat imu_q(state_interfaces_[st_imu(3)].get_value(),
                   state_interfaces_[st_imu(0)].get_value(),
                   state_interfaces_[st_imu(1)].get_value(),
                   state_interfaces_[st_imu(2)].get_value());
        if (imu_q.norm() < 1e-6)
        {
            imu_q = Quat::Identity();
        }
        else
        {
            imu_q.normalize();
        }
        const Vec3 gyro_imu(state_interfaces_[st_imu(4)].get_value(),
                            state_interfaces_[st_imu(5)].get_value(),
                            state_interfaces_[st_imu(6)].get_value());
        ekf_.update_gyro((config_.base_R_imu * gyro_imu).z());

        state.chassis_pose = ekf_.pose();
        state.chassis_twist = ekf_.twist();
        state.gravity_in_base = est::gravity_in_base(imu_q, config_.base_R_imu);

        Wrench raw;
        raw.force = Vec3(state_interfaces_[st_ft(0)].get_value(),
                         state_interfaces_[st_ft(1)].get_value(),
                         state_interfaces_[st_ft(2)].get_value());
        raw.torque = Vec3(state_interfaces_[st_ft(3)].get_value(),
                          state_interfaces_[st_ft(4)].get_value(),
                          state_interfaces_[st_ft(5)].get_value());

        ArmKinematics kin(*model_);
        kin.update(state.arm.q);
        const Iso3 T_chassis_ft = kin.ft_pose();
        const Vec3 gravity_in_ft = T_chassis_ft.linear().transpose() * state.gravity_in_base;
        const Wrench ext_ft = ft_compensator_.update(raw, gravity_in_ft, Vec3::Zero());
        const Mat3 R_odom_ft = (T_odom_chassis(state.chassis_pose) * T_chassis_ft).linear();
        Wrench ext_world;
        ext_world.force = R_odom_ft * ext_ft.force;
        ext_world.torque = R_odom_ft * ext_ft.torque;
        if (compensated_ft)
        {
            *compensated_ft = ext_world;
        }
        state.ext_force_world = ext_world.force;
        state.ext_torque_world = ext_world.torque;

        const ft::ContactResult result = contact_detector_.update(
            state.ext_force_world, Vec3::UnitZ(), ft_compensator_.params().mass, dt);
        if (contact)
        {
            *contact = result;
        }
        state.in_contact = result.in_contact;
        return state;
    }

    void WbcRosController::snapshot_hold(const RobotState& state)
    {
        hold_reference_.chassis_mode = ChassisMode::kTrackPose;
        hold_reference_.chassis_pose = state.chassis_pose;
        hold_reference_.chassis_twist = ChassisTwist{};
        hold_reference_.ee_twist = Twist{};
        hold_reference_.skip_ee = false;
        hold_reference_.ee_frame = EeFrame::kOdom;
        if (state.arm.q.allFinite() && model_) 
        {
            ArmKinematics kin(*model_);
            kin.update(state.arm.q);
            hold_reference_.ee_pose = 
                T_odom_chassis(state.chassis_pose) * kin.tool_pose();
            have_hold_ = true;
        }
    }

    Reference WbcRosController::current_reference(double now_s, const RobotState& state)
    {
        if (state.arm.q.allFinite() && !have_hold_)
        {
            snapshot_hold(state);
        }

        const auto traj = io_.trajectory();  // 从 ROS 消息中读取轨迹
        const bool have_traj = traj && !traj->points.empty();
        const auto twist = io_.teleop(now_s);  // 从 ROS 消息中读取底盘控制速度
        const EeFrame traj_frame =
            have_traj ? traj->points.front().ref.ee_frame : EeFrame::kOdom;

        /*  模式选择  */
        ControlMode next = ControlMode::kHold;

        // 解偶模式：底盘速度控制、机械臂运动轨迹位于机械臂基座标系
        if (twist && have_traj && traj_frame != EeFrame::kOdom)
        {
            next = ControlMode::kTeleopRelative;
        }
        else if (twist)  // 遥控模式：底盘运动，收臂到舒适关节
        {
            next = ControlMode::kChassisTeleop;
        }
        else if (have_traj)  // 全身控制或只有机械臂运动
        {
            next = (traj_frame == EeFrame::kOdom)
                       ? ControlMode::kWholeBody
                       : ControlMode::kArmRelative;
        }

        if (next != mode_)  // 模式切换的时候去复位一下
        {
            snapshot_hold(state);
            mode_ = next;
        }

        Reference ref = hold_reference_;
        ref.skip_ee = false;
        ref.ee_frame = EeFrame::kOdom;

        switch (mode_)
        {
            case ControlMode::kHold: 
                ref.chassis_mode = ChassisMode::kTrackPose;
                ref.ee_twist = Twist{};
                break;

            case ControlMode::kChassisTeleop:
                ref.chassis_mode = ChassisMode::kTrackTwist;
                ref.chassis_twist = *twist;
                ref.skip_ee = true;
                break;

            case ControlMode::kWholeBody:
                ref = sample_reference(traj->points, now_s - traj->start_time);
                ref.chassis_mode = ChassisMode::kFree;
                ref.skip_ee = false;
                ref.ee_frame = EeFrame::kOdom;
                break;

            case ControlMode::kArmRelative:
                ref = sample_reference(traj->points, now_s - traj->start_time);
                ref.chassis_mode = ChassisMode::kTrackPose;
                ref.chassis_pose = hold_reference_.chassis_pose;
                ref.chassis_twist = ChassisTwist{};
                ref.skip_ee = false;
                break;

            case ControlMode::kTeleopRelative:
                ref = sample_reference(traj->points, now_s - traj->start_time);
                ref.chassis_mode = ChassisMode::kTrackTwist;
                ref.chassis_twist = *twist;
                ref.skip_ee = false;
                break;
        }
        return ref;
    }

    void WbcRosController::write_command(const Output& out)
    {
        if (command_interfaces_.size() < static_cast<std::size_t>(kCmdCount))
        {
            return;
        }
        for (int i = 0; i < kNumArmJoints; ++i)
        {
            command_interfaces_[cmd_arm(i, 0)].set_value(out.arm.q[i]);
            command_interfaces_[cmd_arm(i, 1)].set_value(out.arm.dq[i]);
            command_interfaces_[cmd_arm(i, 2)].set_value(out.arm.tau_ff[i]);
            command_interfaces_[cmd_arm(i, 3)].set_value(out.arm.kp[i]);
            command_interfaces_[cmd_arm(i, 4)].set_value(out.arm.kd[i]);
        }
        command_interfaces_[cmd_wheel(0)].set_value(out.wheels.left);
        command_interfaces_[cmd_wheel(1)].set_value(out.wheels.right);
    }

    void WbcRosController::write_hold()
    {
        if (command_interfaces_.size() < static_cast<std::size_t>(kCmdCount)
            || state_interfaces_.size() < static_cast<std::size_t>(kStateCount))
        {
            return;
        }
        ArmVec q = ArmVec::Zero();
        for (int i = 0; i < kNumArmJoints; ++i)
        {
            q[i] = state_interfaces_[st_arm(i, 0)].get_value();
        }
        Quat imu_q(state_interfaces_[st_imu(3)].get_value(),
                   state_interfaces_[st_imu(0)].get_value(),
                   state_interfaces_[st_imu(1)].get_value(),
                   state_interfaces_[st_imu(2)].get_value());
        if (imu_q.norm() < 1e-6)
        {
            imu_q = Quat::Identity();
        }
        else
        {
            imu_q.normalize();
        }
        const Vec3 g = est::gravity_in_base(imu_q, config_.base_R_imu);
        const ArmVec tau_g =
            wbc_ ? wbc_->gravity_torque(q, g) : ArmVec::Zero();
        for (int i = 0; i < kNumArmJoints; ++i)
        {
            command_interfaces_[cmd_arm(i, 0)].set_value(q[i]);
            command_interfaces_[cmd_arm(i, 1)].set_value(0.0);
            command_interfaces_[cmd_arm(i, 2)].set_value(tau_g[i]);
            command_interfaces_[cmd_arm(i, 3)].set_value(config_.command.free_motion.kp[i]);
            command_interfaces_[cmd_arm(i, 4)].set_value(config_.command.free_motion.kd[i]);
        }
        command_interfaces_[cmd_wheel(0)].set_value(0.0);
        command_interfaces_[cmd_wheel(1)].set_value(0.0);
    }

}  // namespace T1::wbc

PLUGINLIB_EXPORT_CLASS(T1::wbc::WbcRosController, controller_interface::ControllerInterface)
