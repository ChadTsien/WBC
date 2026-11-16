# 动力学。重力矩、工具负载。

add_library(dyn STATIC dynamics.cpp)
target_link_libraries(dyn PUBLIC model)
target_defaults(dyn)
