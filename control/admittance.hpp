#pragma once

#include "types/types.hpp"

namespace T1::wbc
{

    struct AdmittanceConfig
    {
        double mass = 2.0;           // kg, virtual
        double damping = 200.0;      // Ns/m; err on the high side against stiff surfaces
        double stiffness = 0.0;      // N/m, 0 = pure force tracking along the normal
        double max_offset = 0.05;    // m
        double max_velocity = 0.05;  // m/s
        double tilt_damping = 5.0;   // Nms/rad, brush axis compliance from contact torque
        double max_tilt = 0.2;       // rad
    };

    // Normal direction: M a + D v + K x = F_measured - F_desired, integrated to an offset
    // added to the brush_tip reference. Tilt: torque about axes perpendicular to the brush
    // axis rotates the reference axis so the brush lies flat on the surface.
    class Admittance
    {
        public:
        void configure(const AdmittanceConfig& config);
        void reset();

        // Returns the position offset [m, world] along `normal`.
        Vec3 update_normal(const Vec3& normal, double measured_force, double desired_force,
                           double dt);
        // Returns a rotation vector [rad, world] to apply to the reference brush axis.
        Vec3 update_tilt(const Vec3& torque_world, const Vec3& brush_axis, double dt);

        Vec3 offset() const { return offset_; }

        private:
        AdmittanceConfig config_;
        double x_ = 0.0, v_ = 0.0;
        Vec3 offset_ = Vec3::Zero();
        Vec3 tilt_ = Vec3::Zero();
    };

}  // namespace T1::wbc
