#pragma once

#include "kine/whole_body.hpp"
#include "control/qp_problem.hpp"

namespace T1::wbc {

struct EeTaskConfig {
  Vec5 kp = Vec5::Constant(5.0);      // 1/s, error feedback gain
  Vec5 weight = Vec5::Constant(1e3);  // per task row
  double max_error_rate = 0.5;        // m/s (and rad/s) cap on kp * error
};

// Rows: x y z velocity, then the two angular-velocity components perpendicular to the
// brush axis (brush_tip z). Cost: weight * || J5 x - (v_ref + kp * e) ||^2.
class EeTrackingTask {
 public:
  void configure(const EeTaskConfig& config);

  // ee_world / ee_ref: brush_tip pose in world. J: whole_body_jacobian().
  void add_to(QpProblem& qp, const WholeBodyJacobian& J, const Iso3& ee_world,
              const Iso3& ee_ref, const Twist& ee_twist_ref);

  const Vec5& last_error() const { return error_; }

 private:
  EeTaskConfig config_;
  Vec5 error_ = Vec5::Zero();
};

}  // namespace T1::wbc
