// WBC 的参考、状态、输出。世界系 = odom，对应 msgs/TaskSpacePoint
#pragma once

#include <cstdint>
#include <vector>

#include "model/chassis_model.hpp"
#include "types/types.hpp"

namespace T1::wbc
{

    /**
     * 底盘任务模式
     *
     * 说明
     * - kFree：底盘少动，只有臂够不着时才跟
     * - kTrackPose：P 控制到参考位姿（差速不能纠横向）
     * - kTrackTwist：跟踪参考 [v, ω]，遥控 / Nav2
     */
    enum class ChassisMode : std::uint8_t
    {
        kFree = 0,        // 机械臂运动优先，底盘辅助(不强制底盘位置)
        kTrackPose = 1,   // 底盘跟踪目标位置(x, y, yaw)
        kTrackTwist = 2   // 底盘跟踪目标速度(v, ω)
    };

    /**
     * 工具接触阶段
     *
     * 说明
     * - kNone：不导纳
     * - kApproach：沿 −法向靠近，直到接触
     * - kClean：恒力清扫
     * - kRetract：力降到 0 再抬起
     */
    enum class ContactPhase : std::uint8_t
    {
        kNone = 0,
        kApproach = 1,
        kClean = 2,
        kRetract = 3
    };

    /**
     * QP 求解状态
     *
     * 说明
     * - kInfeasible / kMaxIter 时控制器保持上一拍指令（速度置 0）
     */
    enum class QpStatus : std::uint8_t
    {
        kOk = 0,
        kInfeasible = 1,
        kMaxIter = 2,
        kNotRun = 3
    };

    /**
     * 一拍任务空间参考，对应 msgs/TaskSpacePoint
     *
     * 成员
     * - chassis_pose / chassis_twist：底盘，odom，m、rad、m/s、rad/s
     *   （消息字段 chassis_*）
     * - ee_pose / ee_twist：brush_tip，odom；绕工具轴滚转不进任务
     * - surface_normal：世界系单位向量，指出表面
     * - desired_normal_force：N，≥ 0，压向表面
     *
     * 说明
     * - 由 sample_reference 从轨迹插值得到
     */
    struct Reference
    {
        ChassisMode chassis_mode = ChassisMode::kFree;
        Pose2d chassis_pose;
        ChassisTwist chassis_twist;
        Iso3 ee_pose = Iso3::Identity();
        Twist ee_twist;
        ContactPhase contact_phase = ContactPhase::kNone;
        Vec3 surface_normal = Vec3::UnitZ();
        double desired_normal_force = 0.0;
    };

    /**
     * 带时间戳的参考点
     *
     * 成员
     * - t：相对轨迹起点，s
     */
    struct TimedReference
    {
        double t = 0.0;
        Reference ref;
    };

    /** 按 t 升序的参考序列；sample_reference 要求非空 */
    using Trajectory = std::vector<TimedReference>;

    /**
     * 控制器一拍看到的机器人状态
     *
     * 成员
     * - chassis_pose / chassis_twist：odom，来自估计器
     * - gravity_in_base：base_link 下的重力加速度，m/s²
     * - ext_force_world / ext_torque_world：补偿后的外力，作用在 brush_tip，世界系
     *
     * 说明
     * - 力已去掉零偏、重力、惯性；接触检测用这一对
     */
    struct RobotState
    {
        ArmState arm;
        Pose2d chassis_pose;
        ChassisTwist chassis_twist;
        Vec3 gravity_in_base = Vec3(0.0, 0.0, -9.81);
        Vec3 ext_force_world = Vec3::Zero();
        Vec3 ext_torque_world = Vec3::Zero();
        bool in_contact = false;
    };

    /**
     * 一拍控制输出
     *
     * 成员
     * - arm：MIT 指令；chassis / wheels：底盘速度与轮速
     * - qp_solution：WbcVec，[v, ω, dq1..dq6]
     * - qp_solve_time_us：ProxSuite run_time，µs
     * - ee_error：5 维工具任务误差（位置 m，轴线 rad）
     * - admittance_offset：工具沿法向的导纳位移，m
     * - manipulability：5 维任务可操作度
     */
    struct Output
    {
        MitCommand arm;
        ChassisTwist chassis;
        WheelSpeeds wheels;
        WbcVec qp_solution = WbcVec::Zero();
        QpStatus qp_status = QpStatus::kNotRun;
        int qp_iterations = 0;
        double qp_solve_time_us = 0.0;
        Vec5 ee_error = Vec5::Zero();
        Vec3 admittance_offset = Vec3::Zero();
        double manipulability = 0.0;
    };

}  // namespace T1::wbc
