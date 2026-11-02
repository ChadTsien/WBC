# Cross compile for Jetson Orin Nano with the Buildroot 2022.08 toolchain (GCC 11.3).
# Libraries are linked from a sysroot mirrored from the Jetson (tools/sync_sysroot.sh),
# so build and target use the exact same ROS / Pinocchio / ProxSuite binaries.
#
#   colcon build --build-base build-aarch64 --install-base install-aarch64 \
#     --cmake-args -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/toolchain-jetson.cmake

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

if(DEFINED ENV{T1_JETSON_TOOLCHAIN})
  set(T1_TOOLCHAIN_DIR "$ENV{T1_JETSON_TOOLCHAIN}")
else()
  set(T1_TOOLCHAIN_DIR "$ENV{HOME}/env/jetson-aarch64-2022.081")
endif()
set(T1_TRIPLE aarch64-buildroot-linux-gnu)

if(DEFINED ENV{T1_SYSROOT})
  set(T1_SYSROOT "$ENV{T1_SYSROOT}")
else()
  get_filename_component(T1_SYSROOT "${CMAKE_CURRENT_LIST_DIR}/../sysroot" ABSOLUTE)
endif()

if(NOT EXISTS "${T1_TOOLCHAIN_DIR}/bin/${T1_TRIPLE}-g++")
  message(FATAL_ERROR "Jetson toolchain not found: ${T1_TOOLCHAIN_DIR}")
endif()
if(NOT EXISTS "${T1_SYSROOT}/usr/lib/aarch64-linux-gnu")
  message(FATAL_ERROR "Sysroot not found: ${T1_SYSROOT}. Run tools/sync_sysroot.sh first.")
endif()

set(CMAKE_C_COMPILER "${T1_TOOLCHAIN_DIR}/bin/${T1_TRIPLE}-gcc")
set(CMAKE_CXX_COMPILER "${T1_TOOLCHAIN_DIR}/bin/${T1_TRIPLE}-g++")
set(CMAKE_SYSROOT "${T1_SYSROOT}")

set(CMAKE_FIND_ROOT_PATH "${T1_SYSROOT}")
set(CMAKE_PREFIX_PATH "/opt/openrobots;/opt/ros/humble")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_EXE_LINKER_FLAGS_INIT
  "-Wl,-rpath-link,${T1_SYSROOT}/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,${T1_SYSROOT}/lib/aarch64-linux-gnu -Wl,-rpath-link,${T1_SYSROOT}/opt/ros/humble/lib -Wl,-rpath-link,${T1_SYSROOT}/opt/openrobots/lib")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT}")
