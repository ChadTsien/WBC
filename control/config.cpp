#include "control/config.hpp"

#include "runtime/yaml_params.hpp"

namespace T1::wbc
{

    WbcConfig load_wbc_config(const std::string& yaml_path)
    {
        // TODO(M1): one small helper per section keeps this under the size budget
        (void)yaml_path;
        return WbcConfig{};
    }

}  // namespace T1::wbc
