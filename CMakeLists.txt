cmake_minimum_required(VERSION 3.16)
project(wbc
  LANGUAGES C CXX
  DESCRIPTION "Whole-body controller"
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(Options)
include(Defaults)
include(Pinocchio)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msgs/msg/TaskSpacePoint.msg
  msgs/msg/TaskSpaceTrajectory.msg
  msgs/msg/WbcStatus.msg
  msgs/msg/ContactState.msg
  msgs/msg/FtStatus.msg
  msgs/srv/TareFt.srv
  msgs/srv/CalibrateFt.srv
  msgs/srv/SetBrush.srv
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# 纯逻辑库。禁止依赖 ROS / MuJoCo / CAN。
add_subdirectory(runtime)
add_subdirectory(model)
add_subdirectory(kine)
add_subdirectory(dyn)
add_subdirectory(signal)
add_subdirectory(estimation)
add_subdirectory(control)

# 仿真库不含 ROS。插件在 ros/。
add_subdirectory(sim)

# 唯一允许出现 ROS 头文件的目录。
add_subdirectory(ros)

install(DIRECTORY
  types runtime model kine dyn signal estimation control sim ros
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS tools/urdf_to_mjcf.py tools/rerun_view.py DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
