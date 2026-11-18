#include "estimation/ekf.hpp"

#include <cmath>

#include <Eigen/Dense>

#include "runtime/math.hpp"

namespace T1::est
{

    namespace
    {
        using State = PlanarEkf::State;
        using Cov = PlanarEkf::Cov;

        template <int M>
        bool joseph_update(State* x, Cov* P, const Eigen::Matrix<double, M, 5>& H,
                           const Eigen::Matrix<double, M, 1>& z,
                           const Eigen::Matrix<double, M, M>& R, double gate_chi2)
        {
            const Eigen::Matrix<double, M, 1> y = z - H * (*x);
            const Eigen::Matrix<double, M, M> S = H * (*P) * H.transpose() + R;
            const double mahal = (y.transpose() * S.inverse() * y)(0, 0);
            if (mahal > gate_chi2)
            {
                return false;
            }
            const Eigen::Matrix<double, 5, M> K = (*P) * H.transpose() * S.inverse();
            *x += K * y;
            (*x)(2) = wrap_angle((*x)(2));
            const Cov I = Cov::Identity();
            const Cov IKH = I - K * H;
            *P = IKH * (*P) * IKH.transpose() + K * R * K.transpose();
            *P = 0.5 * (*P + P->transpose());
            return true;
        }
    }  // namespace

    void PlanarEkf::configure(const EkfConfig& config)
    {
        config_ = config;
    }

    void PlanarEkf::reset(const Pose2d& pose)
    {
        x_.setZero();
        x_(0) = pose.x;
        x_(1) = pose.y;
        x_(2) = wrap_angle(pose.yaw);
        P_ = Cov::Identity();
    }

    void PlanarEkf::predict(double dt)
    {
        if (dt <= 0.0)
        {
            return;
        }
        const double yaw = x_(2);
        const double v = x_(3);
        const double omega = x_(4);
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);

        x_(0) += v * c * dt;
        x_(1) += v * s * dt;
        x_(2) = wrap_angle(yaw + omega * dt);

        Cov F = Cov::Identity();
        F(0, 2) = -v * s * dt;
        F(0, 3) = c * dt;
        F(1, 2) = v * c * dt;
        F(1, 3) = s * dt;
        F(2, 4) = dt;

        Cov Q = Cov::Zero();
        Q(3, 3) = config_.q_v * dt;
        Q(4, 4) = config_.q_omega * dt;
        P_ = F * P_ * F.transpose() + Q;
    }

    bool PlanarEkf::update_odometry(const ChassisTwist& measured)
    {
        Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
        H(0, 3) = 1.0;
        H(1, 4) = 1.0;
        Eigen::Matrix<double, 2, 1> z;
        z << measured.v, measured.omega;
        Eigen::Matrix<double, 2, 2> R = Eigen::Matrix<double, 2, 2>::Zero();
        R(0, 0) = config_.r_v;
        R(1, 1) = config_.r_omega_odom;
        return joseph_update<2>(&x_, &P_, H, z, R, config_.gate_chi2);
    }

    bool PlanarEkf::update_gyro(double omega_z)
    {
        Eigen::Matrix<double, 1, 5> H = Eigen::Matrix<double, 1, 5>::Zero();
        H(0, 4) = 1.0;
        Eigen::Matrix<double, 1, 1> z;
        z << omega_z;
        Eigen::Matrix<double, 1, 1> R;
        R << config_.r_omega_gyro;
        return joseph_update<1>(&x_, &P_, H, z, R, config_.gate_chi2);
    }

    Pose2d PlanarEkf::pose() const
    {
        return Pose2d{x_(0), x_(1), x_(2)};
    }

    ChassisTwist PlanarEkf::twist() const
    {
        return ChassisTwist{x_(3), x_(4)};
    }

}  // namespace T1::est
