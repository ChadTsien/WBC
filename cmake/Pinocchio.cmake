# Pinocchio 2.7（/opt/openrobots）。拒绝 3.x / 4.x，也拒绝 ros-humble-pinocchio。

if(EXISTS "/opt/openrobots")
  list(PREPEND CMAKE_PREFIX_PATH "/opt/openrobots")
endif()

find_package(pinocchio 2.7 QUIET)

if(NOT pinocchio_FOUND)
  message(FATAL_ERROR
    "Pinocchio 2.7 not found.\n"
    "  x86:    ./scripts/setup_deps.sh\n"
    "  Jetson: ./scripts/setup_deps_jetson.sh\n"
    "Then: source scripts/env.sh")
endif()

if(pinocchio_VERSION VERSION_GREATER_EQUAL 3)
  message(FATAL_ERROR
    "Need Pinocchio 2.7.x, found ${pinocchio_VERSION} at ${pinocchio_DIR}. "
    "ros-humble-pinocchio is shadowing /opt/openrobots; source scripts/env.sh last.")
endif()

if(TARGET pinocchio::pinocchio)
  set(PINOCCHIO_LIB pinocchio::pinocchio)
else()
  set(PINOCCHIO_LIB ${pinocchio_LIBRARIES})
endif()

message(STATUS "Pinocchio ${pinocchio_VERSION} (${pinocchio_DIR})")
