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

    /**
     * 平面 EKF，状态 [x, y, yaw, v, ω]，odom 系
     *
     * 说明
     * - 测：轮速里程计的 [v, ω]、陀螺仪 z
     * - map→odom 不进这里，避免定位跳变甩臂
     * - 打滑时里程计更新会被门限拒绝，陀螺仪仍更新
     */
    class PlanarEkf
    {
        public:
        using State = Eigen::Matrix<double, 5, 1>;
        using Cov = Eigen::Matrix<double, 5, 5>;

        void configure(const EkfConfig& config);

        /** 位姿对齐到 pose，v 和 ω 置 0，P 置成单位阵 */
        void reset(const Pose2d& pose);

        /**
         * 恒速预测：沿当前车头积分位置，v、ω 不变
         *
         * 参数
         * - dt：周期，s，须 > 0
         */
        void predict(double dt);

        /**
         * 用轮速里程计的 [v, ω] 更新
         *
         * 返回：新息超过门限（打滑）则为 false，状态不改
         */
        bool update_odometry(const ChassisTwist& measured);

        /**
         * 用陀螺仪 z 轴角速度更新
         *
         * 返回：超过门限则为 false
         */
        bool update_gyro(double omega_z);

        Pose2d pose() const;
        ChassisTwist twist() const;
        const Cov& covariance() const { return P_; }

        private:
        EkfConfig config_;
        State x_ = State::Zero();
        Cov P_ = Cov::Identity();
    };

}  // namespace T1::est
