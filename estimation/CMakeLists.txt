# 底盘里程计、平面 EKF、倾角。不融合 map 系定位。

add_library(estimation STATIC
  odometry.cpp
  ekf.cpp
  tilt.cpp
)
target_link_libraries(estimation PUBLIC kine)
target_defaults(estimation)
