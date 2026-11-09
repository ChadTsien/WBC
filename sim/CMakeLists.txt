# MuJoCo 植物与传感器模型。插件不在这里。
# 实现 world.cpp 时：
include(MuJoCo)

find_package(glfw3 REQUIRED)

add_library(sim STATIC
  world.cpp
  sensors.cpp
  viewer.cpp
)
target_link_libraries(sim PUBLIC kine mujoco_lib glfw GL)
target_defaults(sim)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_world test/test_world.cpp)
  
  target_link_libraries(test_world sim)
  target_defaults(test_world)
  target_compile_definitions(test_world PRIVATE
    T1_TEST_MJCF="${CMAKE_SOURCE_DIR}/config/mjcf/scene.xml")
  
  add_executable(view_world test/view_world.cpp)
  target_link_libraries(view_world sim)
  target_defaults(view_world)
  target_compile_definitions(view_world PRIVATE
    T1_TEST_MJCF="${CMAKE_SOURCE_DIR}/config/mjcf/scene.xml")
endif()
