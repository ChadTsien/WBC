# ros2_control 插件和标定节点。本目录是唯一引用 rclcpp 的地方。

add_library(sim_system SHARED sim_system.cpp)
target_link_libraries(sim_system PUBLIC sim)
ament_target_dependencies(sim_system PUBLIC
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)
target_defaults(sim_system)
pluginlib_export_plugin_description_file(hardware_interface sim_plugin.xml)

add_library(wbc_controller SHARED controller.cpp io.cpp)
target_link_libraries(wbc_controller PUBLIC control "${cpp_typesupport_target}")
ament_target_dependencies(wbc_controller PUBLIC
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  realtime_tools geometry_msgs nav_msgs tf2_msgs)
target_defaults(wbc_controller)
pluginlib_export_plugin_description_file(controller_interface wbc_plugin.xml)

add_executable(ft_calibration ft_calibration.cpp ft_calibration_main.cpp)
target_link_libraries(ft_calibration PUBLIC signal estimation "${cpp_typesupport_target}")
ament_target_dependencies(ft_calibration PUBLIC
  rclcpp geometry_msgs sensor_msgs tf2_ros tf2_eigen)
target_defaults(ft_calibration)

install(TARGETS sim_system wbc_controller ft_calibration
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
