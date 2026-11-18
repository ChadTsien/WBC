#include "control/config.hpp"
#include "control/controller.hpp"
#include "kine/arm.hpp"
#include "model/arm_model.hpp"
#include "types/types.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

namespace
{
    std::string load_urdf()
    {
        std::ifstream in(T1_TEST_URDF);
        if (!in)
        {
            throw std::runtime_error("cannot open " T1_TEST_URDF);
        }
        std::ostringstream ss;
        ss << in.rdbuf();
        return ss.str();
    }

    T1::Iso3 T_odom_tool(const T1::Pose2d& pose, const T1::Iso3& T_chassis_tool)
    {
        T1::Iso3 T_odom_chassis = T1::Iso3::Identity();
        T_odom_chassis.translation() = T1::Vec3(pose.x, pose.y, 0.0);
        T_odom_chassis.linear() =
            Eigen::AngleAxisd(pose.yaw, T1::Vec3::UnitZ()).toRotationMatrix();
        return T_odom_chassis * T_chassis_tool;
    }

    void integrate(T1::wbc::RobotState& state, const T1::WbcVec& x, double dt)
    {
        state.arm.q += x.tail<6>() * dt;
        state.arm.dq = x.tail<6>();
        state.chassis_pose.x += x[0] * std::cos(state.chassis_pose.yaw) * dt;
        state.chassis_pose.y += x[0] * std::sin(state.chassis_pose.yaw) * dt;
        state.chassis_pose.yaw += x[1] * dt;
        state.chassis_twist.v = x[0];
        state.chassis_twist.omega = x[1];
    }
}

TEST(Controller, TracksMovingToolWithin5mm)
{
    T1::ArmModelConfig arm_cfg;
    arm_cfg.urdf_xml = load_urdf();
    T1::ArmModel model(arm_cfg);
    T1::wbc::WbcConfig cfg = T1::wbc::load_wbc_config(T1_TEST_WBC_YAML);
    /* live kFree regularization is high to keep the chassis still; this test needs
       the chassis free to help the arm over 0.3 m */
    cfg.chassis_task.regularization = 10.0;
    T1::wbc::WbcController wbc(cfg, model);

    T1::wbc::RobotState state;
    state.arm.q = cfg.posture.q_nominal;
    wbc.reset(state);

    T1::ArmKinematics kin(model);
    kin.update(state.arm.q);
    const T1::Iso3 T0 = T_odom_tool(state.chassis_pose, kin.tool_pose());

    T1::wbc::Reference ref;
    ref.chassis_mode = T1::wbc::ChassisMode::kFree;
    ref.ee_pose = T0;
    const double vx = 0.03;
    ref.ee_twist.linear = T1::Vec3(vx, 0.0, 0.0);

    const double dt = 0.01;
    const int n = 1000;
    double max_err = 0.0;
    for (int i = 0; i < n; ++i)
    {
        const double t = static_cast<double>(i) * dt;
        ref.ee_pose.translation() = T0.translation() + T1::Vec3(vx * t, 0.0, 0.0);
        const T1::wbc::Output out = wbc.update(state, ref, dt);
        ASSERT_EQ(out.qp_status, T1::wbc::QpStatus::kOk);
        integrate(state, out.qp_solution, dt);
        kin.update(state.arm.q);
        const T1::Vec3 p = T_odom_tool(state.chassis_pose, kin.tool_pose()).translation();
        max_err = std::max(max_err, (p - ref.ee_pose.translation()).norm());
    }
    EXPECT_LT(max_err, 0.005);
}

TEST(Controller, ChassisFollowsWhenToolOutOfReach)
{
    T1::ArmModelConfig arm_cfg;
    arm_cfg.urdf_xml = load_urdf();
    T1::ArmModel model(arm_cfg);
    T1::wbc::WbcConfig cfg = T1::wbc::load_wbc_config(T1_TEST_WBC_YAML);
    
    /* 默认姿态权太小，臂会先被拉奇异；加重后底盘必须跟着走 */
    cfg.posture.weight = T1::ArmVec::Constant(50.0);
    T1::wbc::WbcController wbc(cfg, model);

    T1::wbc::RobotState state;
    state.arm.q = cfg.posture.q_nominal;
    wbc.reset(state);

    T1::ArmKinematics kin(model);
    kin.update(state.arm.q);
    const T1::Iso3 T0 = T_odom_tool(state.chassis_pose, kin.tool_pose());

    T1::wbc::Reference ref;
    ref.chassis_mode = T1::wbc::ChassisMode::kFree;
    ref.ee_pose = T0;
    const double vx = 0.08;
    ref.ee_twist.linear = T1::Vec3(vx, 0.0, 0.0);

    const double dt = 0.01;
    const int n = 1000;
    for (int i = 0; i < n; ++i)
    {
        const double t = static_cast<double>(i) * dt;
        ref.ee_pose.translation() = T0.translation() + T1::Vec3(vx * t, 0.0, 0.0);
        const T1::wbc::Output out = wbc.update(state, ref, dt);
        ASSERT_EQ(out.qp_status, T1::wbc::QpStatus::kOk) << "i=" << i;
        integrate(state, out.qp_solution, dt);
    }
    EXPECT_GT(state.chassis_pose.x, 0.3);
}
