#include "kine/arm.hpp"
#include "kine/whole_body.hpp"
#include "model/arm_model.hpp"
#include "model/chassis_model.hpp"
#include "runtime/math.hpp"
#include "types/types.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
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

    Eigen::Matrix3d Rz(double yaw)
    {
        return Eigen::AngleAxisd(yaw, T1::Vec3::UnitZ()).toRotationMatrix();
    }

    T1::Iso3 world_tool(const T1::Pose2d& pose, const T1::Iso3& tool_in_base)
    {
        T1::Iso3 T_wb = T1::Iso3::Identity();
        T_wb.translation() = T1::Vec3(pose.x, pose.y, 0.0);
        T_wb.linear() = Rz(pose.yaw);
        return T_wb * tool_in_base;
    }
}

TEST(WholeBody, JacobianMatchesFiniteDifference)
{
    // 配置全身模型
    T1::ArmModelConfig cfg;
    cfg.urdf_xml = load_urdf();
    T1::ArmModel arm(cfg);
    T1::ArmKinematics kin(arm);
    const T1::ChassisModel chassis(T1::ChassisModelConfig{});

    T1::ArmVec q;
    q << 0.3, -0.4, 0.1, 0.5, -0.8, 0.2;
    T1::Pose2d pose;
    pose.x = 0.3;
    pose.y = -0.2;
    pose.yaw = 0.5;

    kin.update(q);
    const T1::Iso3 tool0 = kin.tool_pose();
    const T1::WholeBodyJacobian J = 
        T1::whole_body_jacobian(chassis, pose, tool0, kin.tool_jacobian());

    T1::WbcVec x;
    x << 0.2, -0.3, 0.1, -0.2, 0.05, 0.4, -0.3, 0.2;
    const Eigen::Matrix<double, 6, 1> dx = J * x;

    const double dt = 1e-6;
    T1::Pose2d pose1 = pose;
    pose1.x += x[0] * std::cos(pose.yaw) * dt;  // 速度分解
    pose1.y += x[0] * std::sin(pose.yaw) * dt;
    pose1.yaw += x[1] * dt;
    const T1::ArmVec q1 = q + x.tail<6>() * dt;
    kin.update(q1);
    
    const T1::Iso3 T0 = world_tool(pose, tool0);
    const T1::Iso3 T1 = world_tool(pose1, kin.tool_pose());
    const T1::Vec3 dv = (T1.translation() - T0.translation()) / dt;
    const T1::Vec3 dw = T1::so3_log(T1.linear() * T0.linear().transpose()) / dt;
    
    EXPECT_NEAR((dv - dx.head<3>()).norm(), 0.0, 1e-5);
    EXPECT_NEAR((dw - dx.tail<3>()).norm(), 0.0, 1e-5);
}