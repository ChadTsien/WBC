// load_yaml_file / require / require_vector
#include "control/command.hpp"
#include "runtime/yaml_params.hpp"
#include "types/types.hpp"

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include <gtest/gtest.h>
#include <yaml-cpp/node/node.h>

namespace
{
    std::string write_temp_yaml(const std::string& body)
    {
        const std::string path = 
            "/tmp/wbc_yaml_params_" + std::to_string(::getpid()) + ".yaml";
        std::ofstream out(path);
        out << body;
        return path;
    }
}

TEST(YamlParams, LoadAndRead)
{   
    // 写入yaml
    const std::string path = write_temp_yaml(
        "gain: 1.5\n"
        "name: brush\n"
        "limits: [0.1, 0.2, 0.3]\n");
    const YAML::Node root = T1::load_yaml_file(path);
    EXPECT_DOUBLE_EQ(T1::require<double>(root, "gain"), 1.5);
    EXPECT_EQ(T1::require<std::string>(root, "name"), "brush");
    const T1::VecX v = T1::require_vector(root, "limits", 3);
    ASSERT_EQ(v.size(), 3);
    EXPECT_DOUBLE_EQ(v[0], 0.1);
    EXPECT_DOUBLE_EQ(v[1], 0.2);
    EXPECT_DOUBLE_EQ(v[2], 0.3);
    std::remove(path.c_str());
}

TEST(YamlParams, MissingKeyNamesTheKey)
{
    const std::string path = write_temp_yaml("gain: 1.5\n");
    const YAML::Node root = T1::load_yaml_file(path);
    try
    {
        T1::require<double>(root, "stiffness");
        FAIL() << "should throw";
    }
    catch (const std::runtime_error& e)
    {
        EXPECT_NE(std::string(e.what()).find("stiffness"), std::string::npos);
    }
    std::remove(path.c_str());
}

TEST(YamlParams, VectorWrongSize)
{
    const std::string path = write_temp_yaml("limits: [0.1, 0.2]\n");
    const YAML::Node root = T1::load_yaml_file(path);
    try
    {
        T1::require_vector(root, "limits", 3);
        FAIL() << "should throw";
    }
    catch (const std::runtime_error& e)
    {
        EXPECT_NE(std::string(e.what()).find("limits"), std::string::npos);
    }
    std::remove(path.c_str());
}

TEST(YamlParams, MissingFileNamesThePath)
{
    const std::string path = "/tmp/wbc_yaml_params_does_not_exist.yaml";
    try
    {
        T1::load_yaml_file(path);
        FAIL() << "should throw";
    }
    catch (const std::runtime_error& e)
    {
        EXPECT_NE(std::string(e.what()).find(path), std::string::npos);
    }
}