#include "estimation/odometry.hpp"
#include <cmath>
#include "model/chassis_model.hpp"
#include "runtime/math.hpp"
#include "types/types.hpp"

namespace T1::est
{

    DiffDriveOdometry::DiffDriveOdometry(const ChassisModel& chassis) : chassis_(chassis)
    {
    }

    void DiffDriveOdometry::reset(const Pose2d& pose, const WheelSpeeds& wheel_positions)
    {
        pose_ = pose;
        twist_ = ChassisTwist{};
        last_positions_ = wheel_positions;
    }

    void DiffDriveOdometry::update(const WheelSpeeds& wheel_positions, double dt)
    {
        if (dt <= 0.0)
        {
            return;
        }

        const WheelSpeeds delta{wheel_positions.left - last_positions_.left,
                                wheel_positions.right - last_positions_.right};
        const ChassisTwist step = chassis_.to_twist(delta);
        const double ds = step.v;    
        const double dtheta = step.omega;
        const double mid_yaw = pose_.yaw + 0.5 * dtheta;

        pose_.x += ds * std::cos(mid_yaw);
        pose_.y += ds * std::sin(mid_yaw);
        pose_.yaw = wrap_angle(pose_.yaw + dtheta);

        twist_.v = ds / dt;
        twist_.omega = dtheta / dt;
        last_positions_ = wheel_positions;
    }

}  // namespace T1::est
