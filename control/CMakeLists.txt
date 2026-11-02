# 全身 QP。不包含 ROS。ProxSuite 在实现 qp_solver.cpp 时链接：
#   include(ProxSuite)
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
target_link_libraries(control PUBLIC kine dyn signal estimation)
target_defaults(control)
