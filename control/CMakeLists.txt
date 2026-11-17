# 全身 QP。不包含 ROS。ProxSuite 在实现 qp_solver.cpp 时链接：
include(ProxSuite)
#   target_link_libraries(control PRIVATE proxsuite::proxsuite)

add_library(control STATIC
  reference.cpp
  qp_problem.cpp
  qp_solver.cpp
  task_ee.cpp
  task_posture.cpp
  task_chassis.cpp
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

  ament_add_gtest(test_task_ee test/test_task_ee.cpp)
  target_link_libraries(test_task_ee control)
  target_defaults(test_task_ee)

  ament_add_gtest(test_task_posture test/test_task_posture.cpp)
  target_link_libraries(test_task_posture control)
  target_defaults(test_task_posture)

  ament_add_gtest(test_task_chassis test/test_task_chassis.cpp)
  target_link_libraries(test_task_chassis control)
  target_defaults(test_task_chassis)

  ament_add_gtest(test_limits test/test_limits.cpp)
  target_link_libraries(test_limits control)
  target_defaults(test_limits)

  ament_add_gtest(test_command test/test_command.cpp)
  target_link_libraries(test_command control)
  target_defaults(test_command)

  ament_add_gtest(test_config test/test_config.cpp)
  target_link_libraries(test_config control)
  target_defaults(test_config)
  target_compile_definitions(test_config PRIVATE
    T1_TEST_WBC_YAML="${CMAKE_SOURCE_DIR}/config/wbc.yaml")

  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller control)
  target_defaults(test_controller)
  target_compile_definitions(test_controller PRIVATE
    T1_TEST_URDF="${CMAKE_SOURCE_DIR}/model/test/data/t1.urdf"
    T1_TEST_WBC_YAML="${CMAKE_SOURCE_DIR}/config/wbc.yaml")
endif()
