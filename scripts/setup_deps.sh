#!/usr/bin/env bash
# One-shot Ubuntu 22.04 x86_64 development environment for T1-WBC.
# Requires ROS 2 Humble already installed from the official apt source.
#
#   ./scripts/setup_deps.sh
#   source scripts/env.sh
#   ./scripts/check_env.sh
#
# Do not wrap the whole script in sudo. apt uses sudo internally; MuJoCo and
# pip --user must land in the calling user's home. sudo ./scripts/setup_deps.sh
# is tolerated and installs those files for $SUDO_USER.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/versions.sh"

die() { echo "ERROR: $*" >&2; exit 1; }

t1_resolve_dev_user || exit 1
if [[ "$(id -u)" -eq 0 ]]; then
  echo "==> Invoked with sudo; user files go to ${T1_DEV_USER} (${T1_DEV_HOME})"
  run_root() { "$@"; }
  as_dev() { sudo -u "${T1_DEV_USER}" -H -- "$@"; }
else
  run_root() { sudo "$@"; }
  as_dev() { "$@"; }
fi

[[ "$(uname -m)" == "x86_64" ]] || die "x86_64 only. On Jetson use scripts/setup_deps_jetson.sh"
command -v apt-get >/dev/null || die "Need apt (Ubuntu)."
command -v sudo >/dev/null || die "Need sudo."
# shellcheck disable=SC1091
source /etc/os-release
[[ "${VERSION_CODENAME}" == "${T1_UBUNTU_CODENAME}" ]] || die "Need Ubuntu ${T1_UBUNTU_CODENAME}."
[[ -f "/opt/ros/${T1_ROS_DISTRO}/setup.bash" ]] || die "Install ROS 2 ${T1_ROS_DISTRO} first."

echo "==> robotpkg apt source"
if [[ ! -f /etc/apt/keyrings/robotpkg.asc ]]; then
  run_root mkdir -p /etc/apt/keyrings
  curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | run_root tee /etc/apt/keyrings/robotpkg.asc >/dev/null
fi
if [[ ! -f /etc/apt/sources.list.d/robotpkg.list ]]; then
  echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub ${T1_UBUNTU_CODENAME} robotpkg" \
    | run_root tee /etc/apt/sources.list.d/robotpkg.list >/dev/null
fi
run_root cp "${SCRIPT_DIR}/apt-preferences" /etc/apt/preferences.d/wbc
run_root apt-get update

# Do not grep -q a pipeline under pipefail: grep exits at the first hit,
# apt-cache then dies with SIGPIPE, and the version looks missing.
has_exact_version() {
  local listing
  listing="$(apt-cache show "$1" 2>/dev/null || true)"
  grep -Fqx "Version: $2" <<<"${listing}"
}
has_exact_version robotpkg-pinocchio "${T1_PINOCCHIO_VER}" || die "robotpkg-pinocchio ${T1_PINOCCHIO_VER} missing"
has_exact_version robotpkg-hpp-fcl "${T1_HPP_FCL_VER}" || die "robotpkg-hpp-fcl ${T1_HPP_FCL_VER} missing"
has_exact_version robotpkg-proxsuite "${T1_PROXSUITE_APT_VER}" || die "robotpkg-proxsuite ${T1_PROXSUITE_APT_VER} missing"

echo "==> Toolchain, libraries, pinned robotpkg packages"
run_root apt-get install -y \
  build-essential cmake pkg-config curl rsync ca-certificates \
  python3-pip python3-colcon-common-extensions \
  libeigen3-dev libyaml-cpp-dev libgtest-dev libglfw3-dev \
  liburdfdom-dev liburdfdom-headers-dev libconsole-bridge-dev libtinyxml-dev \
  libassimp-dev libqhull-dev liboctomap-dev \
  libboost-filesystem-dev libboost-serialization-dev libboost-system-dev libboost-chrono-dev \
  "robotpkg-pinocchio=${T1_PINOCCHIO_VER}" \
  "robotpkg-hpp-fcl=${T1_HPP_FCL_VER}" \
  "robotpkg-proxsuite=${T1_PROXSUITE_APT_VER}"
run_root apt-mark hold robotpkg-pinocchio robotpkg-hpp-fcl robotpkg-proxsuite

echo "==> ROS 2 packages"
R="ros-${T1_ROS_DISTRO}"
run_root apt-get install -y \
  "${R}-ros2-control" "${R}-ros2-controllers" "${R}-realtime-tools" \
  "${R}-xacro" "${R}-robot-state-publisher" "${R}-joint-state-publisher-gui" \
  "${R}-rviz2" "${R}-tf2-eigen" "${R}-foxglove-bridge" "${R}-rosbag2-storage-mcap"

echo "==> MuJoCo ${T1_MUJOCO_VER} (C library) -> ${T1_MUJOCO_DIR}"
if [[ ! -d "${T1_MUJOCO_DIR}" ]]; then
  as_dev mkdir -p "$(dirname "${T1_MUJOCO_DIR}")"
  url="https://github.com/google-deepmind/mujoco/releases/download/${T1_MUJOCO_VER}/mujoco-${T1_MUJOCO_VER}-linux-x86_64.tar.gz"
  curl -fL "${url}" | as_dev tar -xz -C "$(dirname "${T1_MUJOCO_DIR}")"
fi

echo "==> Python packages (${T1_DEV_USER})"
as_dev pip3 install --user -r "${ROOT_DIR}/requirements.txt"
# rerun-sdk 0.35 声明 numpy>=2。仿真钉死 numpy 1.26.4，所以不让它改 numpy。
as_dev pip3 install --user 'attrs>=23.1' 'pillow>=8' 'pyarrow>=18' 'psutil>=7' 'typing-extensions>=4.5'
as_dev pip3 install --user --no-deps "rerun-sdk==${T1_RERUN_VER}"

echo
echo "Done. Next:"
echo "  source ${SCRIPT_DIR}/env.sh"
echo "  ${SCRIPT_DIR}/check_env.sh"
