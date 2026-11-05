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

  ament_add_gtest(test_rate_limiter test/test_rate_limiter.cpp)
  target_link_libraries(test_rate_limiter runtime)
  target_defaults(test_rate_limiter)

  ament_add_gtest(test_spsc_queue test/test_spsc_queue.cpp)
  target_link_libraries(test_spsc_queue runtime)
  target_defaults(test_spsc_queue)

endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_arm_model test/test_arm_model.cpp)
  target_link_libraries(test_arm_model model)
  target_defaults(test_arm_model)
  target_compile_definitions(test_arm_model PRIVATE
    T1_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/data/t1.urdf")
endif()