# Source in every terminal (or from ~/.bashrc):
#   source scripts/env.sh
#
# Order matters: ROS first, then /opt/openrobots prepended, so Pinocchio 2.7 /
# hpp-fcl 2.4.4 shadow ros-humble-pinocchio (3.x) both at build and run time.

_T1_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${_T1_ROOT}/scripts/versions.sh"

if [ -f "/opt/ros/${T1_ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${T1_ROS_DISTRO}/setup.bash"
fi

if [ -d "${T1_PREFIX}" ]; then
  export PATH="${T1_PREFIX}/bin${PATH:+:$PATH}"
  export PKG_CONFIG_PATH="${T1_PREFIX}/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
  export LD_LIBRARY_PATH="${T1_PREFIX}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  export CMAKE_PREFIX_PATH="${T1_PREFIX}${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
fi

if [ -d "${T1_MUJOCO_DIR}" ]; then
  export MUJOCO_DIR="${T1_MUJOCO_DIR}"
  export LD_LIBRARY_PATH="${MUJOCO_DIR}/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi

# local_setup, not setup: setup.bash re-sources the ROS underlay and would push
# /opt/ros/humble back in front of /opt/openrobots.
if [ -f "${_T1_ROOT}/install/local_setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${_T1_ROOT}/install/local_setup.bash"
fi

export T1_ROOT="${_T1_ROOT}"
unset _T1_ROOT
