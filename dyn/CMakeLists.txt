# 动力学。重力矩、工具负载。

add_library(dyn STATIC dynamics.cpp)
target_link_libraries(dyn PUBLIC model)
target_defaults(dyn)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dynamics test/test_dynamics.cpp)
  target_link_libraries(test_dynamics dyn)
  target_defaults(test_dynamics)
  target_compile_definitions(test_dynamics PRIVATE
    T1_TEST_URDF="${CMAKE_SOURCE_DIR}/model/test/data/t1.urdf")
endif()
