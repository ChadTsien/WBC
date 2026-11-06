# 刚体模型。运动学函数体在 kine/，动力学函数体在 dyn/。
add_library(model STATIC arm_model.cpp chassis_model.cpp)
target_link_libraries(model PUBLIC runtime ${PINOCCHIO_LIB})
target_defaults(model)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_arm_model test/test_arm_model.cpp)
  target_link_libraries(test_arm_model model)
  target_defaults(test_arm_model)
  target_compile_definitions(test_arm_model PRIVATE
    T1_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/test/data/t1.urdf")
endif()
