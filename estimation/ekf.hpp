#pragma once

#include "types/types.hpp"

namespace T1::est
{

    struct EkfConfig
    {
        double q_v = 0.5;            // process noise, (m/s^2)^2 * s
        double q_omega = 1.0;        // (rad/s^2)^2 * s
        double r_v = 1e-4;           // wheel odometry v noise, (m/s)^2
        double r_omega_odom = 1e-2;  // wheel odometry omega noise, (rad/s)^2; large: casters slip
        double r_omega_gyro = 4e-6;  // gyro z noise, (rad/s)^2
        double gate_chi2 = 9.0;      // Mahalanobis gate; rejects slipping-wheel updates
    };

    // State [x, y, yaw, v, omega] in the odom frame. Map-frame localization is not fused
    // here: its jumps would jerk the arm. map -> odom comes from the localization stack.
    class PlanarEkf
    {
        public:
        using State = Eigen::Matrix<double, 5, 1>;
        using Cov = Eigen::Matrix<double, 5, 5>;

        void configure(const EkfConfig& config);
        void reset(const Pose2d& pose);

        void predict(double dt);
        // Returns false if the measurement was rejected by the gate.
        bool update_odometry(const BaseTwist& measured);
        bool update_gyro(double omega_z);

        Pose2d pose() const;
        BaseTwist twist() const;
        const Cov& covariance() const { return P_; }

        private:
        EkfConfig config_;
        State x_ = State::Zero();
        Cov P_ = Cov::Identity();
    };

}  // namespace T1::est
