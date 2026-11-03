#include "signal/contact.hpp"

namespace T1::ft
{

    void ContactDetector::configure(const ContactDetectorConfig& config)
    {
        config_ = config;
    }

    void ContactDetector::reset()
    {
        in_contact_ = false;
        detach_timer_ = 0.0;
    }

    ContactResult ContactDetector::update(const Vec3& /*ext_force_world*/,
                                          const Vec3& /*surface_normal*/, double /*tool_mass*/,
                                          double /*dt*/)
    {
        // TODO(M3b)
        return ContactResult{};
    }

}  // namespace T1::ft
