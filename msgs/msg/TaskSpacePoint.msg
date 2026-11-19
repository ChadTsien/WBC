# One sample of the whole-body reference. Time is relative to TaskSpaceTrajectory.header.stamp.
builtin_interfaces/Duration time_from_start

# Chassis reference, world frame (header.frame_id of the trajectory).
uint8 CHASSIS_FREE = 0
uint8 CHASSIS_TRACK_POSE = 1
uint8 CHASSIS_TRACK_TWIST = 2
uint8 chassis_mode
float64 chassis_x
float64 chassis_y
float64 chassis_yaw
float64 chassis_v
float64 chassis_omega

# brush_tip reference. Roll about the brush axis is ignored (5-DoF task).
uint8 EE_ODOM = 0
uint8 EE_BASE = 1
uint8 EE_MOUNT = 2
uint8 ee_frame
geometry_msgs/Pose ee_pose
geometry_msgs/Twist ee_twist

# Contact reference.
uint8 CONTACT_NONE = 0
uint8 CONTACT_APPROACH = 1
uint8 CONTACT_CLEAN = 2
uint8 CONTACT_RETRACT = 3
uint8 contact_phase
geometry_msgs/Vector3 surface_normal   # unit vector, world frame, pointing out of the surface
float64 desired_normal_force           # N, >= 0, pressing into the surface
