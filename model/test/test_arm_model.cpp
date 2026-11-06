#include "model/arm_model.hpp"
#include "types/types.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <pinocchio/multibody/model.hpp>

namespace
{
    std::string load_urdf()
    {
        std::ifstream in(T1_TEST_URDF);  // 按路径打开文件
        if (!in)
        {
            throw std::runtime_error("cannot open " T1_TEST_URDF);
        }
        std::ostringstream ss;
        ss << in.rdbuf();
        return ss.str();
    } 
}

TEST(ArmModel, LoadsArm)
{
    T1::ArmModelConfig cfg;
    cfg.urdf_xml = load_urdf();
    T1::ArmModel model(cfg);

    EXPECT_EQ(model.model().nq, T1::kNumArmJoints);
    EXPECT_EQ(model.model().nv, T1::kNumArmJoints);
    EXPECT_GE(model.tool_frame_id(), 0);
    EXPECT_GE(model.ft_frame_id(), 0);
    EXPECT_GE(model.mount_frame_id(), 0);
    EXPECT_NO_THROW(model.frame_id("brush_tip"));
    EXPECT_NO_THROW(model.frame_id("ft_sensor"));
    EXPECT_NO_THROW(model.frame_id("arm_base_link"));
}

TEST(ArmModel, LimitsMatchYaml)
{
    T1::ArmModelConfig cfg;
    cfg.urdf_xml = load_urdf();
    T1::ArmModel model(cfg);
    const T1::ArmVec lo = model.lower_limits();
    const T1::ArmVec hi = model.upper_limits();
    const T1::ArmVec ve = model.velocity_limits();

    EXPECT_NEAR(lo[0], -3.14, 1e-9);
    EXPECT_NEAR(hi[0], 3.14, 1e-9);
    EXPECT_NEAR(ve[0], 3.0, 1e-9);
    EXPECT_NEAR(lo[2], -0.34, 1e-9);
    EXPECT_NEAR(hi[2], 0.38, 1e-9);
    EXPECT_NEAR(ve[2], 0.2, 1e-9);
    EXPECT_NEAR(lo[4], -3.33, 1e-9);
    EXPECT_NEAR(hi[5], 1.58, 1e-9);
}

TEST(ArmModel, BadNamesThrow)
{
    T1::ArmModelConfig cfg;
    cfg.urdf_xml = load_urdf();
    cfg.arm_joints[0] = "not_a_joint";
    EXPECT_THROW(T1::ArmModel model(cfg), std::runtime_error);

    T1::ArmModelConfig cfg2;
    cfg2.urdf_xml = load_urdf();
    cfg2.tool_frame = "not_a_frame";
    EXPECT_THROW(T1::ArmModel model(cfg2), std::runtime_error);
    
    T1::ArmModelConfig cfg3;
    cfg3.urdf_xml = load_urdf();
    T1::ArmModel model(cfg3);
    EXPECT_THROW(model.frame_id("nope"), std::runtime_error);
}