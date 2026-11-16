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
            /**
             * 按不等式最大行数分配 C、c_lo、c_hi
             *
             * 说明
             * - add_inequality 累计行数不得超过此值
             */
            explicit QpProblem(int max_inequality_rows);

            /**
             * 清空代价和不等式，盒约束恢复为 ±∞
             *
             * 说明
             * - 每个控制周期求解前调用
             * - H 对角线加 1e-6，保证正定
             */
            void reset();

            /**
             * 累加 weight · ‖A x − b‖²
             *
             * 说明
             * - A 为 n×8，b 为 n
             * - 原公式：1/2 xᵀAᵀAx - 2 (Aᵀb)ᵀx  ->  xᵀH x + g
             * - 展开：H += 2w AᵀA，g −= 2w Aᵀb
             * - weight 用于调整软优先级，任务之间越大越优先
             */
            void add_cost(const MatRef& A, const VecRef& b, double weight);

            /**
             * 按行加权的 ‖A x − b‖²
             *
             * 说明
             * - 第 i 行乘 row_weights[i]
             * - H += 2 Aᵀ W A，g −= 2 Aᵀ W b，W = diag(row_weights)
             */
            void add_cost(const MatRef& A, const VecRef& b, const VecRef& row_weights);

            /**
             * 在 C 末尾追加不等式行
             *
             * 说明
             * - 超出容量返回 false，已有行不变
             * - 有效内容是前 inequality_rows() 行
             */
            bool add_inequality(const MatRef& C, const VecRef& lower, const VecRef& upper);

            /**
             * 与当前盒约束取交集
             *
             * 说明
             * - 下界取更大，上界取更小
             * - 速度 / 加速度 / 位置限位各调用一次
             */
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
