// 全项目共用的类型和维度。改 kNumWbcDof / kTaskDim 会牵动 QP
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace T1
{

    constexpr int kNumArmJoints = 6;
    constexpr int kNumChassisInputs = 2;  // 差速类型底盘：v, omega
    constexpr int kNumWbcDof = kNumChassisInputs + kNumArmJoints;

    /**
     * 末端任务维数
     *
     * 说明
     * - 位置 3 维 + 工具轴线指向 2 维
     * - 工具绕轴滚转无意义，不进任务
     */
    constexpr int kTaskDim = 5;
    
    using Vec3 = Eigen::Vector3d;
    using Vec5 = Eigen::Matrix<double, kTaskDim, 1>;
    using Vec6 = Eigen::Matrix<double, 6, 1>;

    /**
     * 机械臂关节向量，长度 kNumArmJoints
     *
     * 说明
     * - 下标 0..5 对应 arm_joint1..6
     * - J3 为移动副，单位 m；其余转动，单位 rad
     * - 速度、力矩用同一下标，单位见 ArmState
     */
    using ArmVec = Eigen::Matrix<double, kNumArmJoints, 1>;

    /**
     * 全身决策向量，长度 kNumWbcDof = 8
     *
     * 说明
     * - [0]：底盘前向速度 v，m/s，沿 base_link +X
     * - [1]：底盘角速度 ω，rad/s，绕 +Z
     * - [2..7]：六个臂关节速度（rad/s 或 m/s，与 ArmVec 一致）
     */
    using WbcVec = Eigen::Matrix<double, kNumWbcDof, 1>;

    using VecX = Eigen::VectorXd;
    using MatX = Eigen::MatrixXd;
    using Mat3 = Eigen::Matrix3d;
    using Quat = Eigen::Quaterniond;
    using Iso3 = Eigen::Isometry3d;

    /**
     * 力旋量
     *
     * 成员
     * - force：力，N
     * - torque：力矩，N·m
     *
     * 说明
     * - 坐标系由填入方约定（传感器系或世界系）
     */
    struct Wrench
    {
        Vec3 force = Vec3::Zero();
        Vec3 torque = Vec3::Zero();
    };

    /**
     * 空间速度
     *
     * 成员
     * - linear：线速度，m/s
     * - angular：角速度，rad/s
     *
     * 说明
     * - 坐标系由填入方约定
     */
    struct Twist
    {
        Vec3 linear = Vec3::Zero();
        Vec3 angular = Vec3::Zero();
    };

    /**
     * 平面位姿
     *
     * 成员
     * - x / y：位置，m
     * - yaw：偏航，rad
     *
     * 说明
     * - 坐标系由填入方约定；估计器给出的是 odom
     */
    struct Pose2d
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
    };

    /**
     * 差速底盘速度
     *
     * 成员
     * - v：前向速度，m/s，沿 base_link +X
     * - omega：角速度，rad/s，绕 +Z
     *
     * 说明
     * - 不能横移；QP 用这两个量当决策变量，而不是平面三维速度
     */
    struct BaseTwist
    {
        double v = 0.0;      // m/s along base_link +X
        double omega = 0.0;  // rad/s about +Z
    };

    /**
     * 一拍机械臂状态
     *
     * 成员
     * - q：位置（转动 rad，J3 为 m）
     * - dq：速度（rad/s 或 m/s）
     * - tau：力矩或力（N·m 或 N）
     */
    struct ArmState
    {
        ArmVec q = ArmVec::Zero();
        ArmVec dq = ArmVec::Zero();
        ArmVec tau = ArmVec::Zero();
    };

    /**
    * MIT 指令，电机内部计算
    *
    * 成员
    * - q / dq：位置、速度前馈
    * - tau_ff：力矩前馈
    * - kp / kd：刚度、阻尼
    *
    * 说明
    * - τ = kp (q − q_m) + kd (dq − dq_m) + τ_ff
    * - 在电机里算，不是在控制器里算完再发电流
    * - 单位与 ArmState 相同
    */
    struct MitCommand
    {
        ArmVec q = ArmVec::Zero();
        ArmVec dq = ArmVec::Zero();
        ArmVec tau_ff = ArmVec::Zero();
        ArmVec kp = ArmVec::Zero();
        ArmVec kd = ArmVec::Zero();
    };

}  // namespace T1
