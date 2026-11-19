#pragma once

#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <string>
#include <vector>

#include "control/controller.hpp"
#include "estimation/ekf.hpp"
#include "estimation/odometry.hpp"
#include "model/arm_model.hpp"
#include "ros/io.hpp"
#include "signal/compensate.hpp"
#include "signal/contact.hpp"

namespace T1::wbc
{
    /**
     * 整机控制策略（ROS 层）
     *
     * 说明
     * - 不进 QP；切模式时 snapshot_hold，再写出 chassis_mode / skip_ee / ee_frame
     * - kHold：钉上次车位 + 世界系工具
     * - kChassisTeleop：底盘是遥控模式，臂收回舒适构型
     * - kWholeBody：全身控制，车臂协同作业。轨迹在 odom，底盘 kFree
     * - kArmRelative：底盘钉住，只动机械臂
     * - kTeleopRelative：解耦模式，底盘遥控模式，机械臂走轨迹
     */
    enum class ControlMode : std::uint8_t
    {
        kHold = 0,
        kChassisTeleop = 1,
        kWholeBody = 2,
        kArmRelative = 3,
        kTeleopRelative = 4
    };

    /**
     * ros2_control 插件：500 Hz 回路里串起估计、力补偿、接触和全身 QP
     *
     * 说明
     * - 接口按 *_interface_configuration 的顺序借出，.cpp 下标依赖这个顺序
     * - 分配只在 on_configure；update 不再 new
     * - M1 的里程计 / EKF / 力补偿仍是空实现，接线先接上
     */
    class WbcRosController : public controller_interface::ControllerInterface
    {
        public:
            /**
             * 声明参数，与 config/controllers.yaml 对齐
             */
            CallbackReturn on_init() override;

            /**
             * 命令接口名单
             *
             * 说明
             * - 每个臂关节：position / velocity / effort / stiffness / damping
             * - 然后每个轮：velocity
             */
            controller_interface::InterfaceConfiguration command_interface_configuration()
                const override;

            /**
             * 状态接口名单
             *
             * 说明
             * - 臂：position / velocity / effort；轮：position / velocity
             * - 然后 imu 10 个、ft_sensor 6 个
             */
            controller_interface::InterfaceConfiguration state_interface_configuration()
                const override;

            /**
             * 读 yaml 和 URDF，构造模型、WBC、估计器和 io；唯一允许分配
             */
            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

            /**
             * 估计器与 WBC 复位；hold_reference_ 记成当前工具位姿，底盘 kTrackPose
             */
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

            /**
             * 写保持指令：当前 q、自由增益、轮速 0
             */
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

            /**
             * 一拍：换工具 / 标零 → 读状态 → 参考 → QP → 下发 → 发布
             */
            controller_interface::return_type update(const rclcpp::Time& time,
                                                     const rclcpp::Duration& period) override;

        private:
            /**
             * 状态接口 → 里程计 / EKF / 倾角 / 力补偿 → RobotState
             *
             * 参数
             * - compensated_ft：世界系、作用在 brush_tip 的补偿后外力
             * - contact：接触与脱落；M3b 前为默认值
             */
            RobotState read_state(double dt, Wrench* compensated_ft, ft::ContactResult* contact);
            
            /**
             * 把当前车位和世界系工具写入 hold_reference_
             *
             * 说明
             * - 使能、以及 ControlMode 切换时调用
             */
            void snapshot_hold(const RobotState& state);

            /**
             * 当前参考
             *
             * 说明
             * - 遥控且轨迹不在 odom → kTeleopRelative
             * - 只有遥控 → kChassisTeleop；只有轨迹 → kWholeBody / kArmRelative
             * - 都没有 → kHold
             */
            Reference current_reference(double now_s, const RobotState& state);

            /** 把 Output 写进命令接口 */
            void write_command(const Output& out);

            /**
             * 故障 / 失能时的保持
             *
             * 说明
             * - q_des = 实测 q，dq = 0，tau_ff = 重力矩
             * - kp/kd 用自由运动增益；轮速 0
             */
            void write_hold();

            std::vector<std::string> arm_joints_;
            std::vector<std::string> wheel_joints_;
            std::string imu_name_;
            std::string ft_name_;

            WbcConfig config_;
            std::unique_ptr<ArmModel> model_;
            std::unique_ptr<WbcController> wbc_;
            std::unique_ptr<est::DiffDriveOdometry> odometry_;
            est::PlanarEkf ekf_;
            ft::FtCompensator ft_compensator_;
            ft::ContactDetector contact_detector_;
            WbcRosIo io_;
            bool have_hold_ = false;
            ControlMode mode_ = ControlMode::kHold;

            Reference hold_reference_;
    };

}  // namespace T1::wbc
