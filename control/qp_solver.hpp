#pragma once

#include <memory>

#include "control/qp_problem.hpp"
#include "control/types.hpp"

namespace T1::wbc
{

    struct QpSolverConfig
    {
        int max_iterations = 100;
        double eps_abs = 1e-6;
        bool warm_start = true;
    };

    // ProxSuite dense QP wrapper. ProxSuite types stay inside qp_solver.cpp.
    class QpSolver
    {
        public:
        QpSolver(const QpSolverConfig& config, int max_inequality_rows);
        ~QpSolver();
        QpSolver(const QpSolver&) = delete;
        QpSolver& operator=(const QpSolver&) = delete;

        // On failure x is left unchanged.
        QpStatus solve(const QpProblem& problem, WbcVec* x, int* iterations, double* solve_time_us);

        private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}  // namespace T1::wbc
