#include "control/qp_problem.hpp"
#include <limits>

namespace T1::wbc
{

    QpProblem::QpProblem(int max_inequality_rows)
        : C_(MatX::Zero(max_inequality_rows, kNumWbcDof)),
          c_lower_(VecX::Zero(max_inequality_rows)),
          c_upper_(VecX::Zero(max_inequality_rows))
    {
        reset();
    }

    void QpProblem::reset()
    {
        H_.setZero();
        g_.setZero();
        H_.diagonal().array() += 1e-6;
        x_lower_.setConstant(-std::numeric_limits<double>::infinity());
        x_upper_.setConstant(std::numeric_limits<double>::infinity());
        used_rows_ = 0;
    }

    void QpProblem::add_cost(const MatRef& A, const VecRef& b, double weight)
    {
        H_.noalias() += (2.0 * weight) * (A.transpose() * A);
        g_.noalias() -= (2.0 * weight) * (A.transpose() * b);
    }

    void QpProblem::add_cost(const MatRef& A, const VecRef& b,
                             const VecRef& row_weights)
    {
        H_.noalias() += 2.0 * (A.transpose() * row_weights.asDiagonal() * A);
        g_.noalias() -= 2.0 * (A.transpose() * row_weights.cwiseProduct(b));
    }

    bool QpProblem::add_inequality(const MatRef& C, const VecRef& lower,
                                   const VecRef& upper)
    {
        const int n = static_cast<int>(C.rows());   // 这次要追加的行数
        if (used_rows_ + n > C_.rows()) 
        {
            return false;
        }
        C_.middleRows(used_rows_, n) = C;
        c_lower_.segment(used_rows_, n) = lower;
        c_upper_.segment(used_rows_, n) = upper;
        used_rows_ += n;
        return true;
    }

    void QpProblem::intersect_bounds(const WbcVec& lower, const WbcVec& upper)
    {
        x_lower_ = x_lower_.cwiseMax(lower);
        x_upper_ = x_upper_.cwiseMin(upper);
    }

}  // namespace T1::wbc
