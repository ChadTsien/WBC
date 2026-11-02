#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "signal/gravity.hpp"

namespace T1::ft {

// Per-brush gravity parameters persisted in a YAML file (config/brushes.yaml format).
// File I/O: never call from the RT loop.
class BrushRegistry {
 public:
  explicit BrushRegistry(std::string yaml_path);

  bool load(std::string* error);
  bool save(std::string* error) const;

  std::optional<ToolGravityParams> get(const std::string& brush_id) const;
  void set(const std::string& brush_id, const ToolGravityParams& params);
  std::vector<std::string> ids() const;

 private:
  std::string path_;
  std::map<std::string, ToolGravityParams> brushes_;
};

}  // namespace T1::ft
