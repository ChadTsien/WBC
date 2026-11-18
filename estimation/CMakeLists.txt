# 底盘里程计、平面 EKF、倾角。不融合 map 系定位。

add_library(estimation STATIC
  odometry.cpp
  ekf.cpp
  tilt.cpp
)
target_link_libraries(estimation PUBLIC kine)
target_defaults(estimation)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odometry test/test_odometry.cpp)
  target_link_libraries(test_odometry estimation)
  target_defaults(test_odometry)

  ament_add_gtest(test_ekf test/test_ekf.cpp)
  target_link_libraries(test_ekf estimation)
  target_defaults(test_ekf)

  ament_add_gtest(test_tilt test/test_tilt.cpp)
  target_link_libraries(test_tilt estimation)
  target_defaults(test_tilt)
endif()
