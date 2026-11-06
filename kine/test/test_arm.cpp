#include "kine/arm.hpp"
#include "model/arm_model.hpp"
#include "runtime/math.hpp"
#include "types/types.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

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

    T1::ArmModel make_arm()
    {
        T1::ArmModelConfig cfg;
        cfg.urdf_xml = load_urdf();
        return T1::ArmModel(cfg);
    }
}

TEST(ArmKinematics, ZeroPose)
{
    T1::ArmModel arm = make_arm();
    T1::ArmKinematics kin(arm);
    kin.update(T1::ArmVec::Zero());

    const T1::Vec3 mount = kin.frame_pose(arm.mount_frame_id()).translation();
    EXPECT_NEAR(mount.x(), -0.007, 1e-6);
    EXPECT_NEAR(mount.y(), 0.0, 1e-6);
    EXPECT_NEAR(mount.z(), 0.92945, 1e-6);

    const T1::Vec3 tip = kin.tool_pose().translation();
    EXPECT_NEAR(tip.x(), 0.53609984, 1e-6);
    EXPECT_NEAR(tip.y(), 0.00415003, 1e-6);
    EXPECT_NEAR(tip.z(), 0.211047, 1e-6);
}

TEST(ArmKinematics, NumericalJacobian)
{
    T1::ArmModel arm = make_arm();
    T1::ArmKinematics kin(arm);

    T1::ArmVec q;
    q << 0.3, -0.4, 0.1, 0.5, -0.8, 0.2;
    kin.update(q);
    const T1::Iso3 T0 = kin.tool_pose();
    const T1::ArmJacobian J = kin.tool_jacobian();

    const double eps = 1e-6;
    for (int i = 0; i < T1::kNumArmJoints; ++i)
    {
        T1::ArmVec q1 = q;
        q1[i] += eps;   // 加入微小量
        kin.update(q1);
        const T1::Iso3 T1 = kin.tool_pose();
        
        const T1::Vec3 dp = (T1.translation() - T0.translation()) / eps;
        const T1::Vec3 dw = T1::so3_log(T1.linear() * T0.linear().transpose()) / eps;
        EXPECT_NEAR((J.col(i).head<3>() - dp).norm(), 0.0, 1e-5) << "joint " << i;
        EXPECT_NEAR((J.col(i).tail<3>() - dw).norm(), 0.0, 1e-5) << "joint " << i;
    }
}