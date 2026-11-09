#include "estimation/ekf.hpp"

namespace T1::est
{

    void PlanarEkf::configure(const EkfConfig& config)
    {
        config_ = config;
    }

    void PlanarEkf::reset(const Pose2d& /*pose*/)
    {
        // TODO(M2)
    }

    void PlanarEkf::predict(double /*dt*/)
    {
        // TODO(M2)
    }

    bool PlanarEkf::update_odometry(const ChassisTwist& /*measured*/)
    {
        // TODO(M2)
        return false;
    }

    bool PlanarEkf::update_gyro(double /*omega_z*/)
    {
        // TODO(M2)
        return false;
    }

    Pose2d PlanarEkf::pose() const
    {
        // TODO(M2)
        return Pose2d{};
    }

    ChassisTwist PlanarEkf::twist() const
    {
        // TODO(M2)
        return ChassisTwist{};
    }

}  // namespace T1::est
