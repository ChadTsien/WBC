#pragma once

#include "model/robot_model.hpp"
#include "types/types.hpp"

namespace T1
{

    // Arm dynamics terms used as MIT-mode feed-forward. Writes RobotModel::data.
    class Dynamics
    {
        public:
        explicit Dynamics(RobotModel& model);

        // Payload below the F/T sensor (brush etc.), merged into the last arm link.
        // com_in_ft: centre of mass in the ft_sensor frame. Call when the brush changes, not in RT.
        void set_tool_payload(double mass, const Vec3& com_in_ft);

        // gravity_in_base: gravity vector in base_link (accounts for floor slope from the IMU).
        ArmVec gravity_torque(const ArmVec& q, const Vec3& gravity_in_base);

        private:
        RobotModel& model_;
    };

}  // namespace T1
