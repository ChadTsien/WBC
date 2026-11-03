#pragma once

#include "control/types.hpp"

namespace T1::wbc
{

    struct ContactConfig
    {
        double approach_speed = 0.02;   // m/s along -normal until contact
        double force_ramp_rate = 10.0;  // N/s, force reference slope in kClean / kRetract
        double retract_speed = 0.05;    // m/s along +normal once force is ~0
        double released_force = 0.5;    // N, below this kRetract starts lifting
        double max_force = 40.0;        // N, hard cap on the reference
    };

    struct ContactCommand
    {
        double force_ref = 0.0;            // N, pressing into the surface
        double normal_velocity_ref = 0.0;  // m/s along +normal (negative = toward surface)
        bool use_admittance = false;
        bool use_contact_gains = false;  // soft MIT gains
    };

    // Turns the phase requested by the task layer plus the measured contact state into
    // force / velocity references along the surface normal.
    class ContactPhaseManager
    {
        public:
        void configure(const ContactConfig& config);
        void reset();
        ContactCommand update(const Reference& ref, bool in_contact, double measured_force,
                              double dt);

        private:
        ContactConfig config_;
        double force_ref_ = 0.0;
    };

}  // namespace T1::wbc
