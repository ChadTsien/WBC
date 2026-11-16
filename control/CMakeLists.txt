# 全身 QP。不包含 ROS。ProxSuite 在实现 qp_solver.cpp 时链接：
include(ProxSuite)
#   target_link_libraries(control PRIVATE proxsuite::proxsuite)

add_library(control STATIC
  reference.cpp
  qp_problem.cpp
  qp_solver.cpp
  task_ee.cpp
  task_posture.cpp
  task_base.cpp
  limits.cpp
  contact_phase.cpp
  admittance.cpp
  command.cpp
  config.cpp
  controller.cpp
)
target_link_libraries(control PUBLIC kine dyn signal estimation proxsuite::proxsuite)
target_defaults(control)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_qp_problem test/test_qp_problem.cpp)
  target_link_libraries(test_qp_problem control)
  target_defaults(test_qp_problem)

  ament_add_gtest(test_qp_solver test/test_qp_solver.cpp)
  target_link_libraries(test_qp_solver control)
  target_defaults(test_qp_solver)

  ament_add_gtest(test_reference test/test_reference.cpp)
  target_link_libraries(test_reference control)
  target_defaults(test_reference)
endif()
