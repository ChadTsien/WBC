# MuJoCo C 库。写 sim/world.cpp 时再 include，并链接 mujoco_lib。

if(DEFINED ENV{MUJOCO_DIR})
  set(MUJOCO_DIR "$ENV{MUJOCO_DIR}")
else()
  set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.9.0")
endif()

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h PATHS "${MUJOCO_DIR}/include" NO_DEFAULT_PATH)
find_library(MUJOCO_LIBRARY mujoco PATHS "${MUJOCO_DIR}/lib" NO_DEFAULT_PATH)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR
    "MuJoCo not found at ${MUJOCO_DIR}. Run ./scripts/setup_deps.sh, then source scripts/env.sh")
endif()

if(NOT TARGET mujoco_lib)
  add_library(mujoco_lib SHARED IMPORTED)
  set_target_properties(mujoco_lib PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
endif()

message(STATUS "MuJoCo ${MUJOCO_LIBRARY}")
