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
