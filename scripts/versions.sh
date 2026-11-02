# Single source of truth for pinned versions. Sourced by the other scripts.

T1_UBUNTU_CODENAME="jammy"
T1_ROS_DISTRO="humble"
T1_GCC_MAJOR="11"

T1_PINOCCHIO_VER="2.7.0"
T1_HPP_FCL_VER="2.4.4"
T1_PROXSUITE_VER="0.7.3"
T1_PROXSUITE_APT_VER="0.7.3r1"
T1_MUJOCO_VER="3.9.0"
T1_RERUN_VER="0.35.0"

T1_PREFIX="/opt/openrobots"
T1_MUJOCO_DIR="${HOME}/.mujoco/mujoco-${T1_MUJOCO_VER}"
T1_JETSON_TOOLCHAIN="${T1_JETSON_TOOLCHAIN:-${HOME}/env/jetson-aarch64-2022.081}"
T1_JETSON_TRIPLE="aarch64-buildroot-linux-gnu"
