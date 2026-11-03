#include "signal/brushes.hpp"

#include <utility>

namespace T1::ft
{

    BrushRegistry::BrushRegistry(std::string yaml_path) : path_(std::move(yaml_path)) {}

    bool BrushRegistry::load(std::string* error)
    {
        // TODO(M3a)
        if (error) *error = "not implemented";
        return false;
    }

    bool BrushRegistry::save(std::string* error) const
    {
        // TODO(M3a)
        if (error) *error = "not implemented";
        return false;
    }

    std::optional<ToolGravityParams> BrushRegistry::get(const std::string& brush_id) const
    {
        auto it = brushes_.find(brush_id);
        if (it == brushes_.end()) return std::nullopt;
        return it->second;
    }

    void BrushRegistry::set(const std::string& brush_id, const ToolGravityParams& params)
    {
        brushes_[brush_id] = params;
    }

    std::vector<std::string> BrushRegistry::ids() const
    {
        std::vector<std::string> out;
        for (const auto& [id, params] : brushes_) out.push_back(id);
        return out;
    }

}  // namespace T1::ft
