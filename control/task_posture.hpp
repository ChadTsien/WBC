#pragma once

#include "control/qp_problem.hpp"

namespace T1::wbc {

struct PostureTaskConfig {
  ArmVec q_nominal = ArmVec::Zero();       // comfortable configuration, mid-range
  ArmVec weight = ArmVec::Constant(1.0);   // low: only acts in the task null space
  double kp = 1.0;                         // 1/s
};

// Pulls the arm toward q_nominal: weight * || dq - kp (q_nominal - q) ||^2 on arm columns.
// Keeping the arm comfortable is what makes the base follow when the target moves away.
class PostureTask {
 public:
  void configure(const PostureTaskConfig& config);
  void add_to(QpProblem& qp, const ArmVec& q);

 private:
  PostureTaskConfig config_;
};

}  // namespace T1::wbc
