#include "control/qp_solver.hpp"

namespace T1::wbc
{

    struct QpSolver::Impl
    {
        QpSolverConfig config;
        int max_rows = 0;
    };

    QpSolver::QpSolver(const QpSolverConfig& config, int max_inequality_rows)
        : impl_(std::make_unique<Impl>())
    {
        impl_->config = config;
        impl_->max_rows = max_inequality_rows;
        // TODO(M1): create the ProxSuite dense QP once, sized for max_inequality_rows
    }

    QpSolver::~QpSolver() = default;

    QpStatus QpSolver::solve(const QpProblem& /*problem*/, WbcVec* /*x*/, int* /*iterations*/,
                             double* /*solve_time_us*/)
    {
        // TODO(M1)
        return QpStatus::kNotRun;
    }

}  // namespace T1::wbc
