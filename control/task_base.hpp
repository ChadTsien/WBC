#pragma once

#include "control/qp_problem.hpp"
#include "control/types.hpp"

namespace T1::wbc {

struct BaseTaskConfig {
  double regularization = 10.0;  // kFree: cost on [v, omega]; makes the base the slow DoF
  double track_weight = 1e4;     // kTrackTwist / kTrackPose
  double kp_position = 1.0;      // 1/s, kTrackPose
  double kp_yaw = 1.5;           // 1/s, kTrackPose
};

// kFree: base moves only when the arm needs it. kTrackTwist: follow teleop [v, omega].
// kTrackPose: P-control to a twist, then track it (no lateral term: nonholonomic).
class BaseTask {
 public:
  void configure(const BaseTaskConfig& config);
  void add_to(QpProblem& qp, const Reference& ref, const Pose2d& base_pose);

 private:
  BaseTaskConfig config_;
};

}  // namespace T1::wbc
