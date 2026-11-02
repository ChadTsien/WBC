#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace T1 {

constexpr int kNumArmJoints = 6;
constexpr int kNumBaseInputs = 2;  // v, omega
constexpr int kNumWbcDof = kNumBaseInputs + kNumArmJoints;
// x y z + direction of the brush axis (2 DoF); roll about the round brush axis is free.
constexpr int kTaskDim = 5;

using Vec3 = Eigen::Vector3d;
using Vec5 = Eigen::Matrix<double, kTaskDim, 1>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using ArmVec = Eigen::Matrix<double, kNumArmJoints, 1>;
using WbcVec = Eigen::Matrix<double, kNumWbcDof, 1>;
using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;
using Mat3 = Eigen::Matrix3d;
using Quat = Eigen::Quaterniond;
using Iso3 = Eigen::Isometry3d;

struct Wrench {
  Vec3 force = Vec3::Zero();
  Vec3 torque = Vec3::Zero();
};

struct Twist {
  Vec3 linear = Vec3::Zero();
  Vec3 angular = Vec3::Zero();
};

struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct BaseTwist {
  double v = 0.0;      // m/s along base_link +X
  double omega = 0.0;  // rad/s about +Z
};

struct ArmState {
  ArmVec q = ArmVec::Zero();
  ArmVec dq = ArmVec::Zero();
  ArmVec tau = ArmVec::Zero();
};

// Damiao MIT mode, evaluated inside the motor: tau = kp*(q-q_m) + kd*(dq-dq_m) + tau_ff.
struct MitCommand {
  ArmVec q = ArmVec::Zero();
  ArmVec dq = ArmVec::Zero();
  ArmVec tau_ff = ArmVec::Zero();
  ArmVec kp = ArmVec::Zero();
  ArmVec kd = ArmVec::Zero();
};

}  // namespace T1
