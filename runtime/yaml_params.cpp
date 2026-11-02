#include "runtime/yaml_params.hpp"

namespace T1 {

YAML::Node load_yaml_file(const std::string& path) {
  // TODO(M0)
  (void)path;
  return YAML::Node();
}

VecX require_vector(const YAML::Node& /*node*/, const std::string& /*key*/, int /*expected_size*/) {
  // TODO(M0)
  return VecX();
}

}  // namespace T1
