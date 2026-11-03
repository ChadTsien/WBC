#pragma once

#include <memory>
#include <pinocchio/multibody/fwd.hpp>
#include <string>
#include <vector>

#include "types/types.hpp"

namespace T1
{

    struct RobotModelConfig
    {
        std::string urdf_xml;  // expanded URDF text (from /robot_description or a file), not xacro
        std::vector<std::string> arm_joints{"arm_joint1", "arm_joint2", "arm_joint3",
                                            "arm_joint4", "arm_joint5", "arm_joint6"};
        std::string tool_frame = "brush_tip";
        std::string ft_frame = "ft_sensor";
    };

    // Fixed-base arm model rooted at base_link. Wheel joints are locked out of the model;
    // base motion is handled by BaseKinematics. Owns one pinocchio::Data, not thread safe.
    class RobotModel
    {
        public:
        // Throws std::runtime_error on unreadable URDF or unknown joint / frame names.
        explicit RobotModel(const RobotModelConfig& config);
        ~RobotModel();
        RobotModel(const RobotModel&) = delete;
        RobotModel& operator=(const RobotModel&) = delete;

        const pinocchio::Model& model() const;
        pinocchio::Data& data();

        int tool_frame_id() const;
        int ft_frame_id() const;
        // Throws if the frame does not exist. Resolve once at startup, not in the RT loop.
        int frame_id(const std::string& name) const;

        ArmVec lower_limits() const;
        ArmVec upper_limits() const;
        ArmVec velocity_limits() const;

        private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}  // namespace T1
