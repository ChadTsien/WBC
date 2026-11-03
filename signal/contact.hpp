#pragma once

#include "types/types.hpp"

namespace T1::ft
{

    struct ContactDetectorConfig
    {
        double contact_on_force = 2.0;   // N along -normal to declare contact
        double contact_off_force = 1.0;  // N hysteresis to release
        double detach_tolerance = 0.3;   // fraction of tool weight
        double detach_hold_time = 0.1;   // s the detach signature must persist
    };

    struct ContactResult
    {
        bool in_contact = false;
        bool brush_detached = false;
        double normal_force = 0.0;  // N, positive = pressing into the surface
    };

    // If the brush falls off, compensation still subtracts its weight, so the external force
    // reads about +m*g upward with nothing touching: that signature flags a detachment.
    class ContactDetector
    {
        public:
        void configure(const ContactDetectorConfig& config);
        void reset();

        // ext_force_world: compensated force at brush_tip, world frame.
        ContactResult update(const Vec3& ext_force_world, const Vec3& surface_normal,
                             double tool_mass, double dt);

        private:
        ContactDetectorConfig config_;
        bool in_contact_ = false;
        double detach_timer_ = 0.0;
    };

}  // namespace T1::ft
