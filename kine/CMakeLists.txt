# 运动学。模型对象在 model。

add_library(kine STATIC
  arm.cpp
  whole_body.cpp
)
target_link_libraries(kine PUBLIC model)
target_defaults(kine)
