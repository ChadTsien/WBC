#include "control/qp_problem.hpp"

namespace T1::wbc {

QpProblem::QpProblem(int max_inequality_rows)
    : C_(MatX::Zero(max_inequality_rows, kNumWbcDof)),
      c_lower_(VecX::Zero(max_inequality_rows)),
      c_upper_(VecX::Zero(max_inequality_rows)) {
  reset();
}

void QpProblem::reset() {
  // TODO(M1)
}

void QpProblem::add_cost(const MatRef& /*A*/, const VecRef& /*b*/, double /*weight*/) {
  // TODO(M1)
}

void QpProblem::add_cost(const MatRef& /*A*/, const VecRef& /*b*/,
                         const VecRef& /*row_weights*/) {
  // TODO(M1)
}

bool QpProblem::add_inequality(const MatRef& /*C*/, const VecRef& /*lower*/,
                               const VecRef& /*upper*/) {
  // TODO(M1)
  return false;
}

void QpProblem::intersect_bounds(const WbcVec& /*lower*/, const WbcVec& /*upper*/) {
  // TODO(M1)
}

}  // namespace T1::wbc
