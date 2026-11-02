std_msgs/Header header

uint8 QP_OK = 0
uint8 QP_INFEASIBLE = 1
uint8 QP_MAX_ITER = 2
uint8 QP_NOT_RUN = 3
uint8 qp_status
uint32 qp_iterations
float64 qp_solve_time_us
float64 cycle_time_us

float64[5] ee_error              # x y z [m], brush axis direction error 2 [rad], world frame
float64 base_v_cmd               # m/s
float64 base_omega_cmd           # rad/s
float64[] joint_velocity_cmd     # rad/s or m/s, arm joint order
geometry_msgs/Vector3 admittance_offset   # m, world frame
float64 manipulability
string[] active_constraints
