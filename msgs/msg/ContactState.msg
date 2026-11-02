std_msgs/Header header

uint8 phase                      # TaskSpacePoint.CONTACT_*
bool in_contact
bool brush_detached
float64 normal_force             # N, along -surface_normal
geometry_msgs/Wrench wrench_world   # compensated external wrench at brush_tip, world frame
