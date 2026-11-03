// 读 YAML：缺键、长度不对、文件打不开都抛异常，信息里带键名或路径
#pragma once

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

#include "types/types.hpp"

namespace T1
{
    /**
     * 从文件加载 YAML
     *
     * 参数
     * - path：文件路径
     *
     * 返回：根节点
     *
     * 说明
     * - 打不开或解析失败时抛 std::runtime_error，what() 里包含 path
     */
    YAML::Node load_yaml_file(const std::string& path);

    /**
     * 读取必需键
     *
     * 参数
     * - node：YAML 映射
     * - key：键名
     *
     * 返回：转成 T 的值
     *
     * 说明
     * - 键不存在时抛 std::runtime_error，what() 里包含 key
     */
    template <typename T>
    T require(const YAML::Node& node, const std::string& key)
    {
        if (!node[key]) throw std::runtime_error("missing yaml key: " + key);
        return node[key].as<T>();
    }

    /**
     * 读取可选键
     *
     * 参数
     * - fallback：键不存在时的默认值
     */
    template <typename T>
    T get_or(const YAML::Node& node, const std::string& key, const T& fallback)
    {
        return node[key] ? node[key].as<T>() : fallback;
    }

    /**
     * 读取数值序列
     *
     * 参数
     * - key：键名
     * - expected_size：期望长度；< 0 表示不检查
     *
     * 返回：VecX
     *
     * 说明
     * - 缺键、不是序列、长度不对，都抛 std::runtime_error
     */
    VecX require_vector(const YAML::Node& node, const std::string& key, int expected_size = -1);

}  // namespace T1
