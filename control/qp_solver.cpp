#include "control/qp_solver.hpp"
#include "control/types.hpp"
#include "types/types.hpp"

#include <algorithm>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/dense/wrapper.hpp>
#include <proxsuite/proxqp/status.hpp>

namespace T1::wbc
{
    namespace
    {
        constexpr double kBoundInf = 1e20;
        using ProxMat =
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

        void set_timing(int* iterations, double* solve_time_us, int iter, double us)
        {
            if (iterations != nullptr)
            {
                *iterations = iter;
            }
            if (solve_time_us != nullptr)
            {
                *solve_time_us = us;
            }
        }

        QpStatus map_status(proxsuite::proxqp::QPSolverOutput status)
        {
            using Out = proxsuite::proxqp::QPSolverOutput;
            switch (status) 
            {
                case Out::PROXQP_SOLVED:
                    return QpStatus::kOk;
                case Out::PROXQP_MAX_ITER_REACHED:
                    return QpStatus::kMaxIter;
                case Out::PROXQP_PRIMAL_INFEASIBLE:
                case Out::PROXQP_DUAL_INFEASIBLE:
                    return QpStatus::kInfeasible;
                default:
                    return QpStatus::kNotRun;
            }
        }
    }

    struct QpSolver::Impl
    {
        QpSolverConfig config;
        int max_rows = 0;
        bool initialized = false;
        proxsuite::proxqp::dense::QP<double> qp;
        ProxMat H;
        ProxMat C_full;
        VecX g;
        VecX c_lo;
        VecX c_hi;

        Impl(const QpSolverConfig& cfg, int rows)
            : config(cfg),
              max_rows(rows),
              qp(kNumWbcDof, 0, rows, true),
              H(kNumWbcDof, kNumWbcDof),
              C_full(std::max(rows, 0), kNumWbcDof),
              g(kNumWbcDof),
              c_lo(VecX::Constant(std::max(rows, 0), -kBoundInf)),
              c_hi(VecX::Constant(std::max(rows, 0), kBoundInf))
        {
            qp.settings.max_iter = cfg.max_iterations;
            qp.settings.max_iter_in = std::max(cfg.max_iterations, 32);
            qp.settings.eps_abs = cfg.eps_abs;
            qp.settings.verbose = false;
            qp.settings.compute_timings = true;
            qp.settings.initial_guess = 
                cfg.warm_start
                    ? proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT
                    : proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
        }
    };

    QpSolver::QpSolver(const QpSolverConfig& config, int max_inequality_rows)
        : impl_(std::make_unique<Impl>(config, max_inequality_rows))
    {
    }

    QpSolver::~QpSolver() = default;

    QpStatus QpSolver::solve(const QpProblem& problem, WbcVec* x, int* iterations,
                             double* solve_time_us)
    {
        if ((problem.x_lower().array() > problem.x_upper().array()).any()
            || problem.x_lower().hasNaN() || problem.x_upper().hasNaN()
            || !problem.H().allFinite() || !problem.g().allFinite())
        {
            set_timing(iterations, solve_time_us, 0, 0.0);
            return QpStatus::kInfeasible;
        }

        const WbcVec l_box = problem.x_lower().cwiseMax(-kBoundInf);
        const WbcVec u_box = problem.x_upper().cwiseMin(kBoundInf);

        impl_->H = problem.H();
        impl_->g = problem.g();
        if (impl_->max_rows > 0)
        {
            impl_->C_full.setZero();
            impl_->c_lo.setConstant(-kBoundInf);
            impl_->c_hi.setConstant(kBoundInf);
            const int n = problem.inequality_rows();
            if (n > 0)
            {
                impl_->C_full.topRows(n) = problem.C().topRows(n);
                impl_->c_lo.head(n) = problem.c_lower().head(n);
                impl_->c_hi.head(n) = problem.c_upper().head(n);
            }
        }

        auto& qp = impl_->qp;
        const bool has_in = impl_->max_rows > 0;
        // Recompute Ruiz every cycle: H changes with q, and KEEP after a bad
        // first init leaves NaN scales that never recover.
        if (!impl_->initialized)
        {
            if (has_in)
            {
                qp.init(impl_->H, impl_->g, proxsuite::nullopt, proxsuite::nullopt,
                        impl_->C_full, impl_->c_lo, impl_->c_hi, l_box, u_box);
            }
            else
            {
                qp.init(impl_->H, impl_->g, proxsuite::nullopt, proxsuite::nullopt,
                        proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt, l_box,
                        u_box);
            }
            impl_->initialized = true;
        }
        else if (has_in)
        {
            qp.update(impl_->H, impl_->g, proxsuite::nullopt, proxsuite::nullopt,
                      impl_->C_full, impl_->c_lo, impl_->c_hi, l_box, u_box, true);
        }
        else
        {
            qp.update(impl_->H, impl_->g, proxsuite::nullopt, proxsuite::nullopt,
                      proxsuite::nullopt, proxsuite::nullopt, proxsuite::nullopt, l_box,
                      u_box, true);
        }

        qp.solve();

        set_timing(iterations, solve_time_us, static_cast<int>(qp.results.info.iter),
                   qp.results.info.run_time);

        const QpStatus status = map_status(qp.results.info.status);
        if (status == QpStatus::kOk && x != nullptr)
        {
            *x = qp.results.x;
        }
        if (status != QpStatus::kOk)
        {
            // Warm-start from NaN x after a failed solve makes every later
            // well-posed cycle hit 100*(max_iter_in+1)=150100.
            qp.results.cleanup(qp.settings);
            impl_->initialized = false;
        }

        return status;
    }


}  // namespace T1::wbc
