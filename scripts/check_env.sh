#!/usr/bin/env bash
# Read-only environment check. Installs nothing. Exit code 1 if any FAIL.
#
#   source scripts/env.sh && ./scripts/check_env.sh

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/versions.sh"

FAILS=0
ok()   { printf '  [ OK ] %s\n' "$*"; }
warn() { printf '  [WARN] %s\n' "$*"; }
fail() { printf '  [FAIL] %s\n' "$*"; FAILS=$((FAILS + 1)); }

expect_eq() {
  local what="$1" got="$2" want="$3"
  if [[ "${got}" == "${want}" ]]; then ok "${what} ${got}"; else fail "${what}: got '${got}', want '${want}'"; fi
}

ARCH="$(uname -m)"
echo "== System (${ARCH})"
# shellcheck disable=SC1091
source /etc/os-release
expect_eq "Ubuntu" "${VERSION_CODENAME}" "${T1_UBUNTU_CODENAME}"
expect_eq "g++ major" "$(g++ -dumpversion 2>/dev/null | cut -d. -f1)" "${T1_GCC_MAJOR}"
command -v colcon >/dev/null && ok "colcon" || fail "colcon missing"

echo "== ROS 2"
expect_eq "ROS_DISTRO" "${ROS_DISTRO:-unset}" "${T1_ROS_DISTRO}"
for p in hardware_interface controller_interface realtime_tools xacro robot_state_publisher \
         foxglove_bridge rosbag2_storage_mcap; do
  [[ -d "/opt/ros/${T1_ROS_DISTRO}/share/${p}" ]] && ok "${p}" || fail "${p} missing"
done

echo "== Pinned libraries (${T1_PREFIX})"
pc_ver() { PKG_CONFIG_PATH="${T1_PREFIX}/lib/pkgconfig" pkg-config --modversion "$1" 2>/dev/null || echo none; }
expect_eq "pinocchio" "$(pc_ver pinocchio)" "${T1_PINOCCHIO_VER}"
expect_eq "hpp-fcl" "$(pc_ver hpp-fcl)" "${T1_HPP_FCL_VER}"
if [[ -f "${T1_PREFIX}/lib/cmake/proxsuite/proxsuiteConfigVersion.cmake" ]]; then
  got="$(sed -n 's/.*PACKAGE_VERSION "\([0-9.]*\)".*/\1/p' \
    "${T1_PREFIX}/lib/cmake/proxsuite/proxsuiteConfigVersion.cmake" | head -1)"
  expect_eq "proxsuite" "${got}" "${T1_PROXSUITE_VER}"
else
  fail "proxsuite not installed in ${T1_PREFIX}"
fi
if dpkg -s "ros-${T1_ROS_DISTRO}-pinocchio" >/dev/null 2>&1; then
  warn "ros-${T1_ROS_DISTRO}-pinocchio is installed; env.sh must keep ${T1_PREFIX} first"
fi
path_index() {
  local i=0 entry
  IFS=':' read -ra entries <<<"${LD_LIBRARY_PATH:-}"
  for entry in "${entries[@]}"; do
    [[ "${entry}" == "$1" ]] && { echo "${i}"; return; }
    i=$((i + 1))
  done
  echo 9999
}
if [[ $(path_index "${T1_PREFIX}/lib") -lt $(path_index "/opt/ros/${T1_ROS_DISTRO}/lib") ]]; then
  ok "LD_LIBRARY_PATH: ${T1_PREFIX}/lib before /opt/ros/${T1_ROS_DISTRO}/lib"
else
  fail "LD_LIBRARY_PATH: ${T1_PREFIX}/lib must come before /opt/ros (source scripts/env.sh)"
fi

if [[ "${ARCH}" == "x86_64" ]]; then
  echo "== Simulation"
  if [[ -f "${T1_MUJOCO_DIR}/include/mujoco/mujoco.h" ]]; then ok "MuJoCo C ${T1_MUJOCO_VER}"; else fail "MuJoCo C missing at ${T1_MUJOCO_DIR}"; fi
  expect_eq "python mujoco" "$(python3 -c 'import mujoco;print(mujoco.__version__)' 2>/dev/null || echo none)" "${T1_MUJOCO_VER}"
  expect_eq "rerun" "$(python3 -c 'import rerun; print(rerun.__version__)' 2>/dev/null || echo none)" "${T1_RERUN_VER}"

  echo "== Cross compile"
  if [[ -x "${T1_JETSON_TOOLCHAIN}/bin/${T1_JETSON_TRIPLE}-g++" ]]; then
    expect_eq "aarch64 g++ major" "$("${T1_JETSON_TOOLCHAIN}/bin/${T1_JETSON_TRIPLE}-g++" -dumpversion | cut -d. -f1)" "${T1_GCC_MAJOR}"
  else
    warn "Jetson toolchain not found at ${T1_JETSON_TOOLCHAIN}"
  fi
fi

echo
if [[ ${FAILS} -eq 0 ]]; then echo "Environment OK"; else echo "${FAILS} check(s) failed"; exit 1; fi
