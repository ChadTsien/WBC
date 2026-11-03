#pragma once

#include "control/qp_problem.hpp"

namespace T1::wbc
{

    struct LimitsConfig
    {
        ArmVec q_min = ArmVec::Constant(-3.0);
        ArmVec q_max = ArmVec::Constant(3.0);
        ArmVec dq_max = ArmVec::Constant(2.0);
        ArmVec ddq_max = ArmVec::Constant(10.0);
        double v_max = 0.5, omega_max = 1.0;    // m/s, rad/s
        double dv_max = 0.5, domega_max = 1.5;  // m/s^2, rad/s^2
        double position_margin = 0.05;          // rad / m kept away from the hard limits
        double position_gain = 5.0;             // 1/s, how fast the joint may approach a limit
    };

    // All limits become box bounds on x: velocity, acceleration (around x_prev), and
    // position via dq <= gain * (q_max - margin - q).
    class LimitConstraints
    {
        public:
        void configure(const LimitsConfig& config);
        void apply(QpProblem& qp, const ArmVec& q, const WbcVec& x_prev, double dt) const;

        private:
        LimitsConfig config_;
    };

}  // namespace T1::wbc
