# 根 CMakeLists 引入。名字里的 T1_ 是代码里的开关，不出现在文件名上。

option(T1_BUILD_TESTS "Build unit tests" OFF)

# 公开头从仓库根引用：#include "types/types.hpp"
set(T1_INCLUDE_ROOT "${CMAKE_SOURCE_DIR}" CACHE PATH "Public include root")
