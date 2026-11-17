#include "control/command.hpp"
#include "types/types.hpp"

namespace T1::wbc
{

    void CommandBuilder::configure(const CommandConfig& config)
    {
        config_ = config;
    }

    void CommandBuilder::reset(const ArmVec& q_measured)
    {
        q_des_ = q_measured;
    }

    MitCommand CommandBuilder::build(const WbcVec& x, const ArmVec& q_measured,
                                     const ArmVec& tau_gravity, bool contact_gains,
                                     double dt)
    {
        const ArmVec dq = x.tail<6>();
        q_des_ += dq * dt;

        const ArmVec band = ArmVec::Constant(config_.max_tracking_error);
        /*  步进限幅  */
        q_des_ = q_des_.cwiseMax(q_measured - band).cwiseMin(q_measured + band);
        /*  关节限位  */
        q_des_ = q_des_.cwiseMax(config_.q_min).cwiseMin(config_.q_max);

        MitCommand cmd;
        cmd.q = q_des_;
        cmd.dq = dq;
        cmd.tau_ff = tau_gravity;
        const GainSet& g = contact_gains ? config_.contact : config_.free_motion;
        cmd.kp = g.kp;
        cmd.kd = g.kd;
        return cmd;
    }

}  // namespace T1::wbc
