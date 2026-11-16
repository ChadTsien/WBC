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

    /**
     * 解 problem，成功时写入 *x
     *
     * 说明
     * - 失败不改 *x
     * - iterations / solve_time_us 可为 nullptr
     */
    class QpSolver
    {
        public:
            QpSolver(const QpSolverConfig& config, int max_inequality_rows);
            ~QpSolver();
            QpSolver(const QpSolver&) = delete;
            QpSolver& operator=(const QpSolver&) = delete;

            /**
             * 解 problem，成功时写入 *x
             *
             * 说明
             * - x：8 维速度 [v, ω, dq1..dq6]
             * - 失败不改 *x
             * - iterations、solve_time_us（µs）可为 nullptr
             * - max_inequality_rows 须与 problem 构造时一致
             */
            QpStatus solve(const QpProblem& problem, WbcVec* x, int* iterations, double* solve_time_us);

        private:
            struct Impl;
            std::unique_ptr<Impl> impl_;
    };

}  // namespace T1::wbc
