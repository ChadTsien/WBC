#pragma once

#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "types/types.hpp"

namespace T1 {

// Throws std::runtime_error with the file path on parse failure.
YAML::Node load_yaml_file(const std::string& path);

// Throws std::runtime_error naming the missing key.
template <typename T>
T require(const YAML::Node& node, const std::string& key) {
  if (!node[key]) throw std::runtime_error("missing yaml key: " + key);
  return node[key].as<T>();
}

template <typename T>
T get_or(const YAML::Node& node, const std::string& key, const T& fallback) {
  return node[key] ? node[key].as<T>() : fallback;
}

// Reads a YAML sequence of numbers. expected_size < 0 accepts any length;
// otherwise a length mismatch throws.
VecX require_vector(const YAML::Node& node, const std::string& key, int expected_size = -1);

}  // namespace T1
