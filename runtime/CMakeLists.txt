# 滤波、数学、实时工具。零 ROS。

add_library(runtime STATIC
  math.cpp
  filters.cpp
  rate_limiter.cpp
  rt_thread.cpp
  loop_timer.cpp
  yaml_params.cpp
)
target_link_libraries(runtime PUBLIC Eigen3::Eigen yaml-cpp Threads::Threads)
target_defaults(runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_math test/test_math.cpp)
  target_link_libraries(test_math runtime)
  target_defaults(test_math)

  ament_add_gtest(test_yaml_params test/test_yaml_params.cpp)
  target_link_libraries(test_yaml_params runtime)
  target_defaults(test_yaml_params)

  ament_add_gtest(test_filters test/test_filters.cpp)
  target_link_libraries(test_filters runtime)
  target_defaults(test_filters)
endif()