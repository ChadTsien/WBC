# 运动学。模型对象在 model。

add_library(kine STATIC
  arm.cpp
  whole_body.cpp
)
target_link_libraries(kine PUBLIC model)
target_defaults(kine)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_arm test/test_arm.cpp)
  target_link_libraries(test_arm kine)
  target_defaults(test_arm)
  target_compile_definitions(test_arm PRIVATE
    T1_TEST_URDF="${CMAKE_SOURCE_DIR}/model/test/data/t1.urdf")
endif()