#include "runtime/math.hpp"
#include "control/task_ee.hpp"
#include "types/types.hpp"

namespace T1::wbc
{

    void EeTrackingTask::configure(const EeTaskConfig& config)
    {
        config_ = config;
    }

    void EeTrackingTask::add_to(QpProblem& qp, const WholeBodyJacobian& J,
                                const Iso3& ee_world, const Iso3& ee_ref,
                                const Twist& ee_twist_ref, const WbcVec& x_meas)
    {
        const Mat3 R = ee_world.linear();  // 末端姿态
        const Vec3 e_w = R.col(2).cross(ee_ref.linear().col(2));

        // 位置和转角误差
        error_.head<3>() = ee_ref.translation() - ee_world.translation();
        error_(3) = R.col(0).dot(e_w);
        error_(4) = R.col(1).dot(e_w);

        MatX A(kTaskDim, kNumWbcDof);
        A.topRows<3>() = J.topRows<3>();
        A.row(3) = R.col(0).transpose() * J.bottomRows<3>();
        A.row(4) = R.col(1).transpose() * J.bottomRows<3>();

        // kp * error
        const VecX pos_fb = saturate_norm(
            VecX(config_.kp.head<3>().cwiseProduct(error_.head<3>())),
            config_.max_error_rate);
        const VecX ang_fb = saturate_norm(
            VecX(config_.kp.tail<2>().cwiseProduct(error_.tail<2>())),
            config_.max_error_rate);
        
        Vec5 v_ref = Vec5::Zero();
        v_ref.head<3>() = ee_twist_ref.linear;
        v_ref(3) = R.col(0).dot(ee_twist_ref.angular);
        v_ref(4) = R.col(1).dot(ee_twist_ref.angular);

        Vec5 b = v_ref;
        b.head<3>() += pos_fb;
        b.tail<2>() += ang_fb;
        b -= config_.kd.cwiseProduct(Vec5(A * x_meas) - v_ref);

        qp.add_cost(A, b, VecX(config_.weight));
    }

}  // namespace T1::wbc
