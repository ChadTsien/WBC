#include "dyn/dynamics.hpp"
#include "model/arm_model.hpp"
#include "runtime/math.hpp"
#include "types/types.hpp"

#include <cmath>
#include <limits>
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

    T1::ArmModel make_arm()
    {
        T1::ArmModelConfig cfg;
        cfg.urdf_xml = load_urdf();
        return T1::ArmModel(cfg);
    }
}

TEST(Dynamics, LevelGroundVerticalAxesNearZero)
{
    T1::ArmModel model = make_arm();
    T1::Dynamics dyn(model);
    const T1::ArmVec tau =
        dyn.gravity_torque(T1::ArmVec::Zero(), T1::Vec3(0.0, 0.0, -9.81));

    EXPECT_NEAR(tau[0], 0.0, 1e-6);
    EXPECT_NEAR(tau[1], 0.0, 1e-6);
    EXPECT_NEAR(tau[3], 0.0, 1e-6);
    EXPECT_GT(std::abs(tau[2]), 10.0);
}

TEST(Dynamics, TiltedGravityLoadsJoint1)
{
    T1::ArmModel model = make_arm();
    T1::Dynamics dyn(model);
    const double tilt = 10.0 * T1::kPi / 180.0;
    const T1::Vec3 g = Eigen::AngleAxisd(tilt, T1::Vec3::UnitY())
                       * T1::Vec3(0.0, 0.0, -9.81);
    const T1::ArmVec tau = dyn.gravity_torque(T1::ArmVec::Zero(), g);
    EXPECT_GT(std::abs(tau[0]), 1e-3);
}

TEST(Dynamics, NonFiniteQReturnsZero)
{
    T1::ArmModel model = make_arm();
    T1::Dynamics dyn(model);
    T1::ArmVec q = T1::ArmVec::Zero();
    q[2] = std::numeric_limits<double>::quiet_NaN();
    const T1::ArmVec tau = dyn.gravity_torque(q, T1::Vec3(0.0, 0.0, -9.81));
    EXPECT_TRUE(tau.isZero());
}
