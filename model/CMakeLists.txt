# 刚体模型。运动学函数体在 kine/，动力学函数体在 dyn/。

add_library(model STATIC robot_model.cpp)
target_link_libraries(model PUBLIC runtime ${PINOCCHIO_LIB})
target_defaults(model)
