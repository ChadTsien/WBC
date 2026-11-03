#pragma once

#include "runtime/filters.hpp"
#include "signal/gravity.hpp"

namespace T1::ft
{

    struct FtCompensatorConfig
    {
        double sample_hz = 500.0;
        double cutoff_hz = 30.0;
        bool inertia_enabled = false;
        int tare_samples = 250;  // averaged when a tare is requested
    };

    // Real-time chain, one call per control cycle:
    //   raw -> bias & gravity -> inertia (optional) -> low pass -> external wrench (ft_sensor
    //   frame)
    // No allocation after configure().
    class FtCompensator
    {
        public:
        void configure(const FtCompensatorConfig& config);
        void set_params(const ToolGravityParams& params);
        const ToolGravityParams& params() const { return params_; }

        Wrench update(const Wrench& raw, const Vec3& gravity_in_ft, const Vec3& com_accel_in_ft);

        // Starts averaging the next tare_samples readings into a new bias. Caller guarantees
        // the tool is static and in free air until tare_done().
        void request_tare();
        bool tare_done() const { return !taring_; }

        private:
        FtCompensatorConfig config_;
        ToolGravityParams params_;
        FilterBank<Butterworth2, 6> filter_;
        bool taring_ = false;
        int tare_count_ = 0;
        Eigen::Matrix<double, 6, 1> tare_sum_ = Eigen::Matrix<double, 6, 1>::Zero();
    };

}  // namespace T1::ft
