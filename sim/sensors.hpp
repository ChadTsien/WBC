#pragma once

#include <random>

#include "types/types.hpp"

namespace T1::sim {

// Adds the imperfections the real sensors have, so compensation and estimation are
// tested against something harder than ground truth.
struct FtNoiseConfig {
  Wrench bias;                      // constant offset, sensor frame
  double force_noise_std = 0.05;    // N
  double torque_noise_std = 0.002;  // Nm
  double bias_drift_per_s = 0.0;    // N/s random walk, emulates temperature drift
};

class FtSensorModel {
 public:
  FtSensorModel(const FtNoiseConfig& config, unsigned seed);
  Wrench apply(const Wrench& truth, double dt);

 private:
  FtNoiseConfig config_;
  std::mt19937 rng_;
};

struct ImuNoiseConfig {
  double gyro_noise_std = 0.002;   // rad/s
  double gyro_bias_std = 0.001;    // rad/s, drawn once
  double accel_noise_std = 0.02;   // m/s^2
};

class ImuModel {
 public:
  ImuModel(const ImuNoiseConfig& config, unsigned seed);
  Vec3 apply_gyro(const Vec3& truth);
  Vec3 apply_accel(const Vec3& truth);

 private:
  ImuNoiseConfig config_;
  std::mt19937 rng_;
  Vec3 gyro_bias_ = Vec3::Zero();
};

}  // namespace T1::sim
