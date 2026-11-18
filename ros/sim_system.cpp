#include "ros/sim_system.hpp"
#include "model/chassis_model.hpp"
#include "runtime/yaml_params.hpp"
#include "types/types.hpp"

#include <cmath>
#include <csignal>
#include <stdexcept>
#include <string>
#include <vector>
#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/rclcpp.hpp>

namespace
{
    double finite_or_zero(double x)
    {
        return std::isfinite(x) ? x : 0.0;
    }
}

namespace T1::sim
{
    using hardware_interface::CommandInterface;
    using hardware_interface::return_type;
    using hardware_interface::StateInterface;

    SimSystem::CallbackReturn SimSystem::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        {
            return CallbackReturn::ERROR;
        } 
        try 
        {
            const auto& hp = info.hardware_parameters;
            const std::string mjcf = hp.at("mjcf_path");          // config/mjcf/scene.xml 
            const std::string sim_config = hp.at("sim_config");   // config/sim.yaml

            if (info.joints.size() != 8)
            {
                throw std::runtime_error("expected 8 joints (6 arm + 2 wheel)");
            }

            // 给 ros2_control 接口准备内存
            arm_cmd_position_.assign(6, 0.0);
            arm_cmd_velocity_.assign(6, 0.0);
            arm_cmd_effort_.assign(6, 0.0);
            arm_cmd_stiffness_.assign(6, 0.0);
            arm_cmd_damping_.assign(6, 0.0);
            arm_state_position_.assign(6, 0.0);
            arm_state_velocity_.assign(6, 0.0);
            arm_state_effort_.assign(6, 0.0);
            wheel_cmd_velocity_.assign(2, 0.0);
            wheel_state_position_.assign(2, 0.0);
            wheel_state_velocity_.assign(2, 0.0);
            imu_state_.assign(10, 0.0);
            ft_state_.assign(6, 0.0);

            /*  读取  config/sim.yaml 中的参数配置  */ 
            const YAML::Node root = T1::load_yaml_file(sim_config);
            const YAML::Node physics = root["physics"];
            timestep_ = T1::require<double>(physics, "timestep");
            enable_viewer_ = T1::require<bool>(physics, "viewer");

            const YAML::Node imu = root["imu_noise"];  // imu
            ImuNoiseConfig icfg;
            icfg.gyro_noise_std = T1::require<double>(imu, "gyro_noise_std");
            icfg.gyro_bias_std = T1::require<double>(imu, "gyro_bias_std");
            icfg.accel_noise_std = T1::require<double>(imu, "accel_noise_std");
            const unsigned seed = T1::require<unsigned>(root, "seed");
            imu_model_ = std::make_unique<ImuModel>(icfg, seed);

            const YAML::Node ft = root["ft_noise"];    // 六维力传感器
            FtNoiseConfig fcfg;
            const T1::VecX bf = T1::require_vector(ft, "bias_force", 3);
            const T1::VecX bt = T1::require_vector(ft, "bias_torque", 3);
            fcfg.bias.force = T1::Vec3(bf[0], bf[1], bf[2]);
            fcfg.bias.torque = T1::Vec3(bt[0], bt[1], bt[2]);
            fcfg.force_noise_std = T1::require<double>(ft, "force_noise_std");
            fcfg.torque_noise_std = T1::require<double>(ft, "torque_noise_std");
            fcfg.bias_drift_per_s = T1::require<double>(ft, "bias_drift_per_s");
            ft_model_ = std::make_unique<FtSensorModel>(fcfg, seed);

            const std::string limits_path =
            sim_config.substr(0, sim_config.find_last_of('/')) + "/joint_limits.yaml";
            const YAML::Node limits = T1::load_yaml_file(limits_path);
            WorldConfig wcfg;
            wcfg.mjcf_path = mjcf;
            wcfg.timestep = timestep_;
            wcfg.q_home = T1::require_vector(physics, "home_q", T1::kNumArmJoints);
            const char* arms[6] = {
                "arm_joint1", "arm_joint2", "arm_joint3",
                "arm_joint4", "arm_joint5", "arm_joint6"};
            for (int i = 0; i < 6; ++i)
            {
                wcfg.effort_limits[i] = T1::require<double>(limits[arms[i]], "effort");
            }
            world_ = std::make_unique<MujocoWorld>(wcfg);
            world_->reset();
        } 
        catch (const std::exception& e) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("SimSystem"), "on_init: %s", e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    SimSystem::CallbackReturn SimSystem::on_activate(const rclcpp_lifecycle::State&)
    {
        const ArmState arm = world_->arm_state();
        physics_armed_ = false;
        // 先把控制指令赋为当前关节，避免猛冲。kp 保持 0，read() 在收到非零 kp 前不步进。
        for (int i = 0; i < 6; ++i)
        {
            arm_state_position_[i] = arm.q[i];
            arm_cmd_position_[i] = arm.q[i];
            arm_cmd_stiffness_[i] = 0.0;
            arm_cmd_damping_[i] = 0.0;
        }
        if (enable_viewer_)
        {
            viewer_ = std::make_unique<SimViewer>(*world_);
            viewer_->start();
        }
        return CallbackReturn::SUCCESS;
    }

    SimSystem::CallbackReturn SimSystem::on_deactivate(const rclcpp_lifecycle::State&)
    {
        if (viewer_)
        {
            viewer_->stop();
            viewer_.reset();
        }
        return CallbackReturn::SUCCESS;
    }

    std::vector<StateInterface> SimSystem::export_state_interfaces()
    {
        std::vector<StateInterface> out;
        for (size_t i = 0; i < 6; ++i)
        {
            const std::string& n = info_.joints[i].name;
            out.emplace_back(n, "position", &arm_state_position_[i]);
            out.emplace_back(n, "velocity", &arm_state_velocity_[i]);
            out.emplace_back(n, "effort", &arm_state_effort_[i]);
        }
        for (size_t i = 0; i < 2; ++i)
        {
            const std::string& n = info_.joints[6 + i].name;
            out.emplace_back(n, "position", &wheel_state_position_[i]);
            out.emplace_back(n, "velocity", &wheel_state_velocity_[i]);
        }
        const char* imu[] = {
            "orientation.x", "orientation.y", "orientation.z", "orientation.w",
            "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
            "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};
        for (int i = 0; i < 10; ++i)
        {
            out.emplace_back("imu", imu[i], &imu_state_[i]);
        }
        const char* ft[] = {
            "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
        for (int i = 0; i < 6; ++i)
        {
            out.emplace_back("ft_sensor", ft[i], &ft_state_[i]);
        }
        return out;
    }

    std::vector<CommandInterface> SimSystem::export_command_interfaces()
    {
        std::vector<CommandInterface> out;
        for (size_t i = 0; i < 6; ++i)
        {
            const std::string& n = info_.joints[i].name;
            out.emplace_back(n, "position", &arm_cmd_position_[i]);
            out.emplace_back(n, "velocity", &arm_cmd_velocity_[i]);
            out.emplace_back(n, "effort", &arm_cmd_effort_[i]);
            out.emplace_back(n, "stiffness", &arm_cmd_stiffness_[i]);
            out.emplace_back(n, "damping", &arm_cmd_damping_[i]);
        }
        for (size_t i = 0; i < 2; ++i)
        {
            const std::string& n = info_.joints[6 + i].name;
            out.emplace_back(n, "velocity", &wheel_cmd_velocity_[i]);
        }
        return out;
    }

    return_type SimSystem::read(const rclcpp::Time&, const rclcpp::Duration& period)
    {
        const int n = physics_armed_
            ? std::max(1, static_cast<int>(std::lround(period.seconds() / timestep_)))
            : 0;
        for (int i = 0; i < n; ++i)
        {
            world_->step();
        }

        // read arm state in sim
        const ArmState arm = world_->arm_state();
        for (int i = 0; i < 6; ++i)
        {
            arm_state_position_[i] = arm.q[i];
            arm_state_velocity_[i] = arm.dq[i];
            arm_state_effort_[i] = arm.tau[i];
        }
        
        // read chassis state in sim
        const WheelSpeeds q = world_->wheel_positions();
        const WheelSpeeds dq = world_->wheel_speeds();
        wheel_state_position_[0] = q.left;
        wheel_state_position_[1] = q.right;
        wheel_state_velocity_[0] = dq.left;
        wheel_state_velocity_[1] = dq.right;

        // read imu in sim
        const Quat quat = world_->imu_orientation();
        imu_state_[0] = quat.x();
        imu_state_[1] = quat.y();
        imu_state_[2] = quat.z();
        imu_state_[3] = quat.w();
        const Vec3 gyro = imu_model_->apply_gyro(world_->imu_angular_velocity());
        const Vec3 acc = imu_model_->apply_accel(world_->imu_linear_acceleration());
        imu_state_[4] = gyro.x();
        imu_state_[5] = gyro.y();
        imu_state_[6] = gyro.z();
        imu_state_[7] = acc.x();
        imu_state_[8] = acc.y();
        imu_state_[9] = acc.z();

        // read six forcesense in sim
        const Wrench ft = ft_model_->apply(world_->ft_raw(), n * timestep_);
        ft_state_[0] = ft.force.x();
        ft_state_[1] = ft.force.y();
        ft_state_[2] = ft.force.z();
        ft_state_[3] = ft.torque.x();
        ft_state_[4] = ft.torque.y();
        ft_state_[5] = ft.torque.z();
        return return_type::OK;
    }

    return_type SimSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
    {
        MitCommand cmd;
        for (int i = 0; i < 6; ++i) 
        {
            cmd.q[i] = finite_or_zero(arm_cmd_position_[i]);
            cmd.dq[i] = finite_or_zero(arm_cmd_velocity_[i]);
            cmd.tau_ff[i] = finite_or_zero(arm_cmd_effort_[i]);
            cmd.kp[i] = finite_or_zero(arm_cmd_stiffness_[i]);
            cmd.kd[i] = finite_or_zero(arm_cmd_damping_[i]);
        }
        world_->set_arm_command(cmd);

        WheelSpeeds wheels;
        wheels.left = finite_or_zero(wheel_cmd_velocity_[0]);
        wheels.right = finite_or_zero(wheel_cmd_velocity_[1]);
        world_->set_wheel_command(wheels);

        if (!physics_armed_)
        {
            for (int i = 0; i < 6; ++i)
            {
                if (cmd.kp[i] > 0.0)
                {
                    physics_armed_ = true;
                    break;
                }
            }
        }
        return return_type::OK;
    }

}  // namespace T1::sim

PLUGINLIB_EXPORT_CLASS(T1::sim::SimSystem, hardware_interface::SystemInterface)
