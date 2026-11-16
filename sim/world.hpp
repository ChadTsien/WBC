#pragma once

#include <memory>
#include <string>
#include <mutex>

#include "model/chassis_model.hpp"
#include "types/types.hpp"

namespace T1::sim
{
    /**
     * MuJoCo 世界配置
     *
     * 成员
     * - mjcf_path：展开后的 MJCF 路径（scene.xml）
     * - timestep：物理步长，s，必须与 MJCF 里 <option timestep> 一致
     * - effort_limits：六轴力矩饱和，N·m 或 N（J3）
     */
    struct WorldConfig
    {
        std::string mjcf_path;
        double timestep = 0.001;  // s, must match <option timestep> in the MJCF
        ArmVec effort_limits = ArmVec::Constant(10.0);
    };

    /**
     * 仿真真值，只给评估和 viewer
     *
     * 说明
     * - 估计器运行时不要读这里
     * - tool_pose_world 在世界系，不是 base_link
     */
    struct GroundTruth
    {
        Pose2d chassis_pose;
        ChassisTwist chassis_twist;
        Iso3 tool_pose_world = Iso3::Identity();
    };

    /**
     * MuJoCo 世界（独占 mjModel / mjData）
     *
     * 说明
     * - 所有 mj_* 只出现在 world.cpp
     * - 非线程安全；viewer 用同一把锁读 data
     */
    class MujocoWorld
    {
        public:
            /**
             * 加载 MJCF 并按名字解析关节 / 执行器 / site
             *
             * 说明
             * - 路径空、XML 失败或名字对不上时抛 runtime_error
             */
            explicit MujocoWorld(const WorldConfig& config);  
            ~MujocoWorld();
            MujocoWorld(const MujocoWorld&) = delete;
            MujocoWorld& operator=(const MujocoWorld&) = delete;

            void reset();

            /**
             * 推进一拍物理
             *
             * 说明
             * - 每步按最新 MIT 指令重算力矩，再夹到 effort_limits
             * - 轮速写入 velocity 执行器
             */
            void step();
            double time() const;

            void set_arm_command(const MitCommand& command);
            void set_wheel_command(const WheelSpeeds& speeds);

            ArmState arm_state() const;
            WheelSpeeds wheel_speeds() const;
            WheelSpeeds wheel_positions() const;  // 累积转角，rad
            Wrench ft_raw() const;                // 传感器系，无噪声
            Quat imu_orientation() const;
            Vec3 imu_angular_velocity() const;
            Vec3 imu_linear_acceleration() const;

            /**
             * 仿真真值（世界系）
             *
             * 说明
             * - 底盘 x/y/yaw、[v, ω]，工具 Iso3
             * - 只给测试和 Rerun；估计器运行时不要读
             */
            GroundTruth ground_truth() const;

        private:
            friend class SimViewer;

            std::mutex& mutex() const;
            void* raw_model() const;  // mjModel*，只给 SimViewer
            void* raw_data() const;   // mjData*

            struct Impl;
            std::unique_ptr<Impl> impl_;
    };

}  // namespace T1::sim
