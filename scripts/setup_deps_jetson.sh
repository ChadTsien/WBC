#!/usr/bin/env bash
# One-shot Jetson Orin Nano (JetPack 6, Ubuntu 22.04 aarch64) runtime/build environment.
# robotpkg has no arm64 binaries, so Pinocchio / hpp-fcl / ProxSuite are built from
# source at the same pinned versions into the same prefix as on x86.
#
#   ./scripts/setup_deps_jetson.sh
#   source scripts/env.sh
#   ./scripts/check_env.sh
#
# Do not wrap the whole script in sudo. apt / cmake --install use sudo internally.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/versions.sh"

die() { echo "ERROR: $*" >&2; exit 1; }

t1_resolve_dev_user || exit 1
if [[ "$(id -u)" -eq 0 ]]; then
  echo "==> Invoked with sudo; source builds go to ${T1_DEV_USER} (${T1_DEV_HOME})"
  run_root() { "$@"; }
  as_dev() { sudo -u "${T1_DEV_USER}" -H -- "$@"; }
else
  run_root() { sudo "$@"; }
  as_dev() { "$@"; }
fi

[[ "$(uname -m)" == "aarch64" ]] || die "aarch64 only. On x86 use scripts/setup_deps.sh"
[[ -f "/opt/ros/${T1_ROS_DISTRO}/setup.bash" ]] || die "Install ROS 2 ${T1_ROS_DISTRO} first."
command -v sudo >/dev/null || die "Need sudo."

# Orin Nano has 8 GB shared memory; Pinocchio template code needs ~2 GB per job.
JOBS="${JOBS:-2}"
SRC_DIR="${SRC_DIR:-${T1_DEV_HOME}/t1_deps_src}"

echo "==> System packages"
R="ros-${T1_ROS_DISTRO}"
run_root apt-get update
run_root apt-get install -y \
  build-essential cmake pkg-config git rsync \
  python3-colcon-common-extensions \
  libeigen3-dev libyaml-cpp-dev libgtest-dev \
  liburdfdom-dev liburdfdom-headers-dev libconsole-bridge-dev libtinyxml-dev \
  libassimp-dev libqhull-dev liboctomap-dev \
  libboost-filesystem-dev libboost-serialization-dev libboost-system-dev libboost-chrono-dev \
  "${R}-ros2-control" "${R}-ros2-controllers" "${R}-realtime-tools" \
  "${R}-xacro" "${R}-robot-state-publisher" "${R}-tf2-eigen" \
  "${R}-foxglove-bridge" "${R}-rosbag2-storage-mcap"

build_from_git() {
  local name="$1" url="$2" tag="$3"
  shift 3
  if [[ -f "${T1_PREFIX}/lib/cmake/${name}/${name}Config.cmake" ]]; then
    echo "==> ${name} already installed, skip"
    return
  fi
  echo "==> Building ${name} ${tag}"
  as_dev mkdir -p "${SRC_DIR}"
  [[ -d "${SRC_DIR}/${name}" ]] || as_dev git clone --recursive --branch "${tag}" --depth 1 "${url}" "${SRC_DIR}/${name}"
  as_dev cmake -S "${SRC_DIR}/${name}" -B "${SRC_DIR}/${name}/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="${T1_PREFIX}" \
    -DCMAKE_PREFIX_PATH="${T1_PREFIX}" \
    -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_TESTING=OFF "$@"
  as_dev cmake --build "${SRC_DIR}/${name}/build" -j"${JOBS}"
  run_root cmake --install "${SRC_DIR}/${name}/build"
}

build_from_git hpp-fcl https://github.com/humanoid-path-planner/hpp-fcl.git "v${T1_HPP_FCL_VER}" \
  -DHPP_FCL_HAS_QHULL=ON
build_from_git pinocchio https://github.com/stack-of-tasks/pinocchio.git "v${T1_PINOCCHIO_VER}" \
  -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=ON
build_from_git proxsuite https://github.com/Simple-Robotics/proxsuite.git "v${T1_PROXSUITE_VER}" \
  -DBUILD_WITH_VECTORIZATION_SUPPORT=OFF

echo
echo "Done. Next:"
echo "  source ${SCRIPT_DIR}/env.sh"
echo "  ${SCRIPT_DIR}/check_env.sh"
