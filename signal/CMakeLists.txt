# 力信号：重力补偿、滤波、接触与脱落。离开本目录后力视为已补偿。

add_library(signal STATIC
  gravity.cpp
  identify.cpp
  inertia.cpp
  compensate.cpp
  brushes.cpp
  contact.cpp
)
target_link_libraries(signal PUBLIC runtime)
target_defaults(signal)
