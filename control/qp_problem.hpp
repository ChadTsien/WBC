#pragma once

#include "types/types.hpp"

namespace T1::wbc
{

    using WbcMat = Eigen::Matrix<double, kNumWbcDof, kNumWbcDof>;
    /** 绑到已有矩阵，避免实时循环里为 MatX 分配临时对象 */
    using MatRef = Eigen::Ref<const MatX>;
    using VecRef = Eigen::Ref<const VecX>;

    /**
     * 速度级 QP 的代价与约束累积器
     *
     * 说明
     * - 形式：min ½ xᵀHx + gᵀx，s.t. x_lo ≤ x ≤ x_hi，c_lo ≤ Cx ≤ c_hi
     * - x = [v, ω, dq1..dq6]
     * - 存储只在构造时分配；之后每个周期 reset 再填，可进实时循环
     */
    class QpProblem
    {
        public:
            explicit QpProblem(int max_inequality_rows);

            // Clears costs, inequalities and resets bounds to +-infinity.
            void reset();

            // Adds weight * || A x - b ||^2 (A has kNumWbcDof columns).
            void add_cost(const MatRef& A, const VecRef& b, double weight);
            void add_cost(const MatRef& A, const VecRef& b, const VecRef& row_weights);

            // Appends rows; returns false if max_inequality_rows would be exceeded.
            bool add_inequality(const MatRef& C, const VecRef& lower, const VecRef& upper);

            // Intersects the current box with [lower, upper].
            void intersect_bounds(const WbcVec& lower, const WbcVec& upper);

            const WbcMat& H() const { return H_; }
            const WbcVec& g() const { return g_; }
            int inequality_rows() const { return used_rows_; }
            // Top inequality_rows() rows are valid.
            const MatX& C() const { return C_; }
            const VecX& c_lower() const { return c_lower_; }
            const VecX& c_upper() const { return c_upper_; }
            const WbcVec& x_lower() const { return x_lower_; }
            const WbcVec& x_upper() const { return x_upper_; }

        private:
            WbcMat H_ = WbcMat::Zero();
            WbcVec g_ = WbcVec::Zero();
            MatX C_;
            VecX c_lower_, c_upper_;
            WbcVec x_lower_, x_upper_;
            int used_rows_ = 0;
    };

}  // namespace T1::wbc
