#include "control/qp_problem.hpp"
#include "control/task_ee.hpp"
#include "kine/whole_body.hpp"
#include "types/types.hpp"

#include <cmath>
#include <gtest/gtest.h>

TEST(TaskEe, ZeroErrorZeroTwistGivesZeroB)
{
    T1::wbc::EeTrackingTask task;
    task.configure({});
    T1::wbc::QpProblem qp(0);

    const T1::WholeBodyJacobian J = T1::WholeBodyJacobian::Zero();
    const T1::Iso3 T = T1::Iso3::Identity();
    const T1::Twist twist;
    task.add_to(qp, J, T, T, twist);

    EXPECT_NEAR(task.last_error().norm(), 0.0, 1e-12);
    EXPECT_NEAR(qp.g().norm(), 0.0, 1e-12);
}

TEST(TaskEe, PureRollAroundToolZHasZeroAxisError)
{
    T1::wbc::EeTrackingTask task;
    task.configure({});
    T1::wbc::QpProblem qp(0);

    const T1::WholeBodyJacobian J = T1::WholeBodyJacobian::Zero();
    const T1::Iso3 world = T1::Iso3::Identity();

    T1::Iso3 ref = T1::Iso3::Identity();
    ref.linear() = Eigen::AngleAxisd(0.4, T1::Vec3::UnitZ()).toRotationMatrix();
    const T1::Twist twist;

    task.add_to(qp, J, world, ref, twist);

    EXPECT_NEAR(task.last_error().head<3>().norm(), 0.0, 1e-12);
    EXPECT_NEAR(task.last_error().tail<2>().norm(), 0.0, 1e-12);
}

TEST(TaskEe, AxisTiltPointOneRadian)
{
    T1::wbc::EeTrackingTask task;
    task.configure({});
    T1::wbc::QpProblem qp(0);

    const T1::WholeBodyJacobian J = T1::WholeBodyJacobian::Zero();
    const T1::Iso3 world = T1::Iso3::Identity();
    T1::Iso3 ref = T1::Iso3::Identity();
    ref.linear() = Eigen::AngleAxisd(0.1, T1::Vec3::UnitX()).toRotationMatrix();
    const T1::Twist twist;

    task.add_to(qp, J, world, ref, twist);
    
    EXPECT_NEAR(task.last_error().head<3>().norm(), 0.0, 1e-12);
    EXPECT_NEAR(task.last_error()(3), std::sin(0.1), 1e-12);
    EXPECT_NEAR(task.last_error()(4), 0.0, 1e-12);
    EXPECT_NEAR(task.last_error().tail<2>().norm(), 0.1, 1e-3);
}

TEST(TaskEe, DampingOpposesMeasuredTaskVelocity)
{
    T1::wbc::EeTaskConfig cfg;
    cfg.kd = T1::Vec5::Constant(1.0);
    cfg.weight = T1::Vec5::Ones();
    T1::wbc::EeTrackingTask task;
    task.configure(cfg);
    T1::wbc::QpProblem qp(0);

    T1::WholeBodyJacobian J = T1::WholeBodyJacobian::Zero();
    J(0, 0) = 1.0;
    const T1::Iso3 T = T1::Iso3::Identity();
    const T1::Twist twist;
    T1::WbcVec x_meas = T1::WbcVec::Zero();
    x_meas[0] = 0.2;

    task.add_to(qp, J, T, T, twist, x_meas);

    EXPECT_NEAR(task.last_error().norm(), 0.0, 1e-12);
    EXPECT_NEAR(qp.g()[0], 0.4, 1e-12);
}