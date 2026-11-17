#include "control/config.hpp"

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>

TEST(Config, LoadsRepoWbcYaml)
{
    const T1::wbc::WbcConfig c = T1::wbc::load_wbc_config(T1_TEST_WBC_YAML);
    EXPECT_DOUBLE_EQ(c.control_rate_hz, 500.0);
    EXPECT_EQ(c.max_inequality_rows, 64);
    EXPECT_DOUBLE_EQ(c.ee.kp[0], 5.0);
    EXPECT_DOUBLE_EQ(c.ee.weight[3], 300.0);
    EXPECT_DOUBLE_EQ(c.posture.q_nominal[1], 1.0);
    EXPECT_DOUBLE_EQ(c.limits.v_max, 0.5);
    EXPECT_DOUBLE_EQ(c.command.free_motion.kp[2], 2000.0);
    EXPECT_DOUBLE_EQ(c.chassis.wheel_radius, 0.0655);
    EXPECT_TRUE(c.base_R_imu.isIdentity());
    EXPECT_EQ(c.default_brush, "default");
}

TEST(Config, MissingKeyNamesTheKey)
{
    const std::string path =
        "/tmp/wbc_config_missing_" + std::to_string(::getpid()) + ".yaml";
    {
        std::ofstream out(path);
        out << "control_rate_hz: 500.0\nmax_inequality_rows: 64\n";
    }
    try
    {
        T1::wbc::load_wbc_config(path);
        FAIL() << "should throw";
    }
    catch (const std::runtime_error& e)
    {
        EXPECT_NE(std::string(e.what()).find("ee_task"), std::string::npos);
    }
    std::remove(path.c_str());
}
