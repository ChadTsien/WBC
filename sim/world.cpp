#include "sim/world.hpp"

namespace T1::sim {

struct MujocoWorld::Impl {
  WorldConfig config;
  MitCommand arm_command;
  WheelSpeeds wheel_command;
};

MujocoWorld::MujocoWorld(const WorldConfig& config) : impl_(std::make_unique<Impl>()) {
  impl_->config = config;
  // TODO(M0): mj_loadXML, mj_makeData, resolve joint / actuator / sensor ids by name
}

MujocoWorld::~MujocoWorld() = default;

void MujocoWorld::reset() {
  // TODO(M0)
}

void MujocoWorld::step() {
  // TODO(M0)
}

double MujocoWorld::time() const {
  // TODO(M0)
  return 0.0;
}

void MujocoWorld::set_arm_command(const MitCommand& command) { impl_->arm_command = command; }

void MujocoWorld::set_wheel_command(const WheelSpeeds& speeds) { impl_->wheel_command = speeds; }

ArmState MujocoWorld::arm_state() const {
  // TODO(M0)
  return ArmState{};
}

WheelSpeeds MujocoWorld::wheel_speeds() const {
  // TODO(M0)
  return WheelSpeeds{};
}

WheelSpeeds MujocoWorld::wheel_positions() const {
  // TODO(M0)
  return WheelSpeeds{};
}

Wrench MujocoWorld::ft_raw() const {
  // TODO(M3a)
  return Wrench{};
}

Quat MujocoWorld::imu_orientation() const {
  // TODO(M2)
  return Quat::Identity();
}

Vec3 MujocoWorld::imu_angular_velocity() const {
  // TODO(M2)
  return Vec3::Zero();
}

Vec3 MujocoWorld::imu_linear_acceleration() const {
  // TODO(M2)
  return Vec3::Zero();
}

GroundTruth MujocoWorld::ground_truth() const {
  // TODO(M0)
  return GroundTruth{};
}

}  // namespace T1::sim
