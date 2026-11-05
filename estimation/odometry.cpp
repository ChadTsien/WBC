#include "estimation/odometry.hpp"

namespace T1::est
{

    DiffDriveOdometry::DiffDriveOdometry(const ChassisModel& chassis) : chassis_(chassis)
    {
    }

    void DiffDriveOdometry::reset(const Pose2d& pose, const WheelSpeeds& wheel_positions)
    {
        pose_ = pose;
        twist_ = BaseTwist{};
        last_positions_ = wheel_positions;
    }

    void DiffDriveOdometry::update(const WheelSpeeds& /*wheel_positions*/, double /*dt*/)
    {
        // TODO(M2)
    }

}  // namespace T1::est
