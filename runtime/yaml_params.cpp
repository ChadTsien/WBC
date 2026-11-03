// 读 YAML：缺键、长度不对、文件打不开都抛异常，信息里带键名或路径
#include "runtime/yaml_params.hpp"
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>

#include <exception>
#include <stdexcept>
#include <string>
#include "types/types.hpp"

namespace T1
{
    YAML::Node load_yaml_file(const std::string& path)
    {
        try
        {
            return YAML::LoadFile(path);
       }
       catch (const std::exception& e)
       {
            throw std::runtime_error("failed to load yaml '" + path + "': " + e.what());
       }
    }

    VecX require_vector(const YAML::Node& node, const std::string& key, int expected_size)
    {
        if (!node[key])
        {
            throw std::runtime_error("missing yaml key: " + key);
        }
        const YAML::Node seq = node[key];
        if (!seq.IsSequence())
        {
            throw std::runtime_error("yaml key is not a sequence: " + key);
        }
        const int n = static_cast<int>(seq.size());
        if (expected_size >= 0 && n != expected_size)
        {
            throw std::runtime_error("yaml key '" + key + "' size mismatch");
        }
        VecX out(n);
        for (int i = 0; i < n; ++i)
        {
            out[i] = seq[i].as<double>();
        }
        return out;
    }

}  // namespace T1
