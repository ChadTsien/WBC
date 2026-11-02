# One sample of the whole-body reference. Time is relative to TaskSpaceTrajectory.header.stamp.
builtin_interfaces/Duration time_from_start

# Base reference, world frame (header.frame_id of the trajectory).
uint8 BASE_FREE = 0
uint8 BASE_TRACK_POSE = 1
uint8 BASE_TRACK_TWIST = 2
uint8 base_mode
float64 base_x
float64 base_y
float64 base_yaw
float64 base_v
float64 base_omega

# brush_tip reference, world frame. Roll about the brush axis is ignored (5-DoF task).
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
