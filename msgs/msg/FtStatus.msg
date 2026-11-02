std_msgs/Header header

uint8 CALIB_NONE = 0
uint8 CALIB_OK = 1
uint8 CALIB_RUNNING = 2
uint8 CALIB_FAILED = 3
uint8 calib_state

string brush_id
float64 tool_mass                # kg, everything below the sensor
geometry_msgs/Vector3 tool_com   # m, sensor frame
geometry_msgs/Wrench bias        # sensor frame
