# MuJoCo 植物与传感器模型。插件不在这里。
# 实现 world.cpp 时：
#   include(MuJoCo)
#   target_link_libraries(sim PRIVATE mujoco_lib)

add_library(sim STATIC
  world.cpp
  sensors.cpp
  viewer.cpp
)
target_link_libraries(sim PUBLIC kine)
target_defaults(sim)
