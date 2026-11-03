#pragma once

#include <cstdint>
#include <vector>

#include "kine/base.hpp"
#include "types/types.hpp"

namespace T1::wbc
{

    enum class BaseMode : std::uint8_t
    {
        kFree = 0,
        kTrackPose = 1,
        kTrackTwist = 2
    };
    enum class ContactPhase : std::uint8_t
    {
        kNone = 0,
        kApproach = 1,
        kClean = 2,
        kRetract = 3
    };
    enum class QpStatus : std::uint8_t
    {
        kOk = 0,
        kInfeasible = 1,
        kMaxIter = 2,
        kNotRun = 3
    };

    // Mirrors msgs/TaskSpacePoint. World = odom frame.
    struct Reference
    {
        BaseMode base_mode = BaseMode::kFree;
        Pose2d base_pose;
        BaseTwist base_twist;
        Iso3 ee_pose = Iso3::Identity();
        Twist ee_twist;
        ContactPhase contact_phase = ContactPhase::kNone;
        Vec3 surface_normal = Vec3::UnitZ();
        double desired_normal_force = 0.0;
    };

    struct TimedReference
    {
        double t = 0.0;  // s from trajectory start
        Reference ref;
    };

    using Trajectory = std::vector<TimedReference>;

    struct RobotState
    {
        ArmState arm;
        Pose2d base_pose;  // odom frame, from the estimator
        BaseTwist base_twist;
        Vec3 gravity_in_base = Vec3(0.0, 0.0, -9.81);
        Vec3 ext_force_world = Vec3::Zero();  // compensated, at brush_tip
        Vec3 ext_torque_world = Vec3::Zero();
        bool in_contact = false;
    };

    struct Output
    {
        MitCommand arm;
        BaseTwist base;
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
