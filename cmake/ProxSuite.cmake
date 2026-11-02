# ProxSuite，写 control/qp_solver.cpp 时再 include。

if(EXISTS "/opt/openrobots")
  list(PREPEND CMAKE_PREFIX_PATH "/opt/openrobots")
endif()

find_package(proxsuite QUIET)

if(NOT proxsuite_FOUND)
  message(FATAL_ERROR
    "ProxSuite not found in /opt/openrobots.\n"
    "  x86:    ./scripts/setup_deps.sh\n"
    "  Jetson: ./scripts/setup_deps_jetson.sh")
endif()

message(STATUS "ProxSuite ${proxsuite_VERSION} (${proxsuite_DIR})")
