#pragma once

#include "model/chassis_model.hpp"
#include "types/types.hpp"

namespace T1::est
{

    // Integrates wheel encoder positions into a pose in the odom frame.
    class DiffDriveOdometry
    {
        public:
        explicit DiffDriveOdometry(const ChassisModel& chassis);

        void reset(const Pose2d& pose, const WheelSpeeds& wheel_positions);
        // wheel_positions: accumulated wheel angles [rad]; dt > 0.
        void update(const WheelSpeeds& wheel_positions, double dt);

        Pose2d pose() const { return pose_; }
        BaseTwist twist() const { return twist_; }

        private:
        ChassisModel chassis_;
        Pose2d pose_;
        BaseTwist twist_;
        WheelSpeeds last_positions_;
    };

}  // namespace T1::est
