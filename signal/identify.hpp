#pragma once

#include <vector>

#include "signal/gravity.hpp"

namespace T1::ft
{

    // One static pose: averaged raw reading and the gravity direction at that pose.
    struct CalibrationSample
    {
        Wrench raw;
        Vec3 gravity_in_ft = Vec3::Zero();
    };

    struct IdentificationResult
    {
        bool ok = false;
        ToolGravityParams params;
        double rms_force = 0.0;   // N
        double rms_torque = 0.0;  // Nm
        double condition = 0.0;   // condition number of the regressor; large = poses too similar
    };

    // Linear least squares for the 10 unknowns [m, m*c (3), F_bias (3), T_bias (3)].
    // Only pitch changes gravity in the sensor frame on this arm: spread poses over > 90 deg.
    class GravityIdentifier
    {
        public:
        void add(const CalibrationSample& sample);
        void clear() { samples_.clear(); }
        std::size_t size() const { return samples_.size(); }

        // Fails (ok = false) with fewer than 3 samples or condition above max_condition.
        IdentificationResult solve(double max_condition = 1e3) const;

        // Bias only, mass and CoM kept from `known`. Needs >= 1 sample.
        IdentificationResult solve_bias(const ToolGravityParams& known) const;

        private:
        std::vector<CalibrationSample> samples_;
    };

}  // namespace T1::ft
