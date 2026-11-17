#include "control/limits.hpp"
#include <algorithm>
#include "types/types.hpp"

namespace T1::wbc
{

    void LimitConstraints::configure(const LimitsConfig& config)
    {
        config_ = config;
    }

    void LimitConstraints::apply(QpProblem& qp, const ArmVec& q, const WbcVec& x_prev,
                                 double dt) const
    {
        /*  速度硬约束  */
        WbcVec vel_lo;
        WbcVec vel_hi;
        vel_lo[0] = -config_.v_max;
        vel_hi[0] = config_.v_max;
        vel_lo[1] = -config_.omega_max;
        vel_hi[1] = config_.omega_max;
        vel_lo.tail<6>() = -config_.dq_max;
        vel_hi.tail<6>() = config_.dq_max;
        qp.intersect_bounds(vel_lo, vel_hi);

        /*  加速度硬约束  */
        WbcVec acc_max;
        acc_max[0] = config_.dv_max;
        acc_max[1] = config_.domega_max;
        acc_max.tail<6>() = config_.ddq_max;
        qp.intersect_bounds(x_prev - dt * acc_max, x_prev + dt * acc_max);

        /* 位置硬约束 */
        const double inf = std::numeric_limits<double>::infinity();
        WbcVec pos_lo = WbcVec::Constant(-inf);
        WbcVec pos_hi = WbcVec::Constant(inf);
        pos_hi.tail<6>() = config_.position_gain 
            * (config_.q_max.array() - config_.position_margin - q.array());
        pos_lo.tail<6>() = config_.position_gain
            * (config_.q_min.array() + config_.position_margin - q.array());

        for (int i = 0; i < kNumArmJoints; ++i)
        {
            const int k = 2 + i;
            const double lo = std::max(qp.x_lower()[k], pos_lo[k]);
            const double hi = std::min(qp.x_upper()[k], pos_hi[k]);

            if (lo > hi)
            {
                if (q[i] > config_.q_max[i] - config_.position_margin)
                {
                    pos_lo[k] = -inf;
                    pos_hi[k] = std::max(pos_hi[k], qp.x_lower()[k]);
                }
                else
                {
                    pos_hi[k] = inf;
                    pos_lo[k] = std::min(pos_lo[k], qp.x_upper()[k]);
                }
            }
        }
        qp.intersect_bounds(pos_lo, pos_hi);
    }

}  // namespace T1::wbc
