#include "control/contact_phase.hpp"

namespace T1::wbc
{

    void ContactPhaseManager::configure(const ContactConfig& config)
    {
        config_ = config;
    }

    void ContactPhaseManager::reset()
    {
        force_ref_ = 0.0;
    }

    ContactCommand ContactPhaseManager::update(const Reference& /*ref*/, bool /*in_contact*/,
                                               double /*measured_force*/, double /*dt*/)
    {
        // TODO(M3b)
        return ContactCommand{};
    }

}  // namespace T1::wbc
