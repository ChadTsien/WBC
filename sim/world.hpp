#pragma once

#include <memory>
#include <string>

#include "kine/base.hpp"
#include "types/types.hpp"

namespace T1::sim
{

    struct WorldConfig
    {
        std::string mjcf_path;
        double timestep = 0.001;  // s, must match <option timestep> in the MJCF
        ArmVec effort_limits = ArmVec::Constant(10.0);
    };

    // Ground truth from the simulator; the estimator never reads this except for evaluation.
    struct GroundTruth
    {
        Pose2d base_pose;
        BaseTwist base_twist;
        Iso3 tool_pose_world = Iso3::Identity();
    };

    // Owns mjModel / mjData. All MuJoCo calls stay in mujoco_world.cpp.
    class MujocoWorld
    {
        public:
        explicit MujocoWorld(const WorldConfig& config);  // throws on load failure
        ~MujocoWorld();
        MujocoWorld(const MujocoWorld&) = delete;
        MujocoWorld& operator=(const MujocoWorld&) = delete;

        void reset();
        // One physics step. MIT torques are recomputed every step from the latest command,
        // emulating the motor's internal loop, then clamped to effort_limits.
        void step();
        double time() const;

        void set_arm_command(const MitCommand& command);
        void set_wheel_command(const WheelSpeeds& speeds);

        ArmState arm_state() const;
        WheelSpeeds wheel_speeds() const;
        WheelSpeeds wheel_positions() const;  // rad, accumulated
        Wrench ft_raw() const;                // sensor frame, no noise
        Quat imu_orientation() const;
        Vec3 imu_angular_velocity() const;
        Vec3 imu_linear_acceleration() const;
        GroundTruth ground_truth() const;

        private:
        friend class SimViewer;
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}  // namespace T1::sim
